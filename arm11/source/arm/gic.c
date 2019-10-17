/*
 *   This file is part of GodMode9
 *   Copyright (C) 2017-2019 Wolfvak
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common.h>
#include <arm.h>

#include "arm/gic.h"

/* Generic Interrupt Controller Registers */
#define REG_GIC(cpu, o, t)  REG_ARM_PMR(0x200 + ((cpu) * 0x100) + (o), t)

#define REG_GIC_CONTROL(c)  (*REG_GIC(c, 0x00, u32))
#define REG_GIC_PRIOMASK(c) (*REG_GIC(c, 0x04, u32))
#define REG_GIC_POI(c)      (*REG_GIC(c, 0x08, u32))
#define REG_GIC_IRQACK(c)   (*REG_GIC(c, 0x0C, u32))
#define REG_GIC_IRQEND(c)   (*REG_GIC(c, 0x10, u32))
#define REG_GIC_LASTPRIO(c) (*REG_GIC(c, 0x14, u32))
#define REG_GIC_PENDING(c)  (*REG_GIC(c, 0x18, u32))

#define GIC_THIS_CPU_ALIAS  (-1)
#define GIC_IRQ_SPURIOUS    (1023)

/* Interrupt Distributor Registers */
#define REG_DIC(off, type)  REG_ARM_PMR(0x1000 + (off), type)

#define REG_DIC_CONTROL     (*REG_DIC(0x00, u32))
#define REG_DIC_TYPE        (*REG_DIC(0x04, u32))
#define REG_DIC_SETENABLE   REG_DIC(0x100, u32)
#define REG_DIC_CLRENABLE   REG_DIC(0x180, u32)
#define REG_DIC_SETPENDING  REG_DIC(0x200, u32)
#define REG_DIC_CLRPENDING  REG_DIC(0x280, u32)
#define REG_DIC_PRIORITY    REG_DIC(0x400, u8)
#define REG_DIC_TARGETPROC  REG_DIC(0x800, u8)
#define REG_DIC_CFGREG      REG_DIC(0xC00, u32)
#define REG_DIC_SOFTINT     (*REG_DIC(0xF00, u32))

/* Interrupt Handling */
#define LOCAL_IRQS  (32)
#define DIC_MAX_IRQ (LOCAL_IRQS + MAX_IRQ)

#define IRQN_IS_LOCAL(n)    ((n) < LOCAL_IRQS)
#define IRQN_IS_VALID(n)    ((n) < DIC_MAX_IRQ)

#define LOCAL_IRQ_OFF(c, n) (((c) * LOCAL_IRQS) + (n))
#define GLOBAL_IRQ_OFF(n)   ((MAX_CPU * LOCAL_IRQS) + (n))

#define IRQ_TABLE_OFF(c, n) \
    (IRQN_IS_LOCAL(n) ? LOCAL_IRQ_OFF(c, n) : GLOBAL_IRQ_OFF(n))

static GIC_IntHandler IRQ_Handlers[GLOBAL_IRQ_OFF(MAX_IRQ)];

static GIC_IntHandler GIC_GetCB(u32 irqn, u32 cpu)
{
    //irqn &= ~(15 << 10); // clear source CPU bits
    return IRQ_Handlers[IRQ_TABLE_OFF(cpu, irqn)];
}

static void GIC_SetCB(u32 irqn, u32 cpumask, GIC_IntHandler handler)
{
    if (IRQN_IS_LOCAL(irqn)) {
        while(cpumask) {
            u32 cpu = top_bit(cpumask);
            IRQ_Handlers[LOCAL_IRQ_OFF(cpu, irqn)] = handler;
            cpumask &= ~BIT(cpu);
        }
    } else {
        IRQ_Handlers[GLOBAL_IRQ_OFF(irqn)] = handler;
    }
}

static void GIC_ClearCB(u32 irqn, u32 cpumask)
{
    GIC_SetCB(irqn, cpumask, NULL);
}

void __attribute__((section(".vectors"))) GIC_MainHandler(void)
{
    u32 cpu = ARM_CoreID();
    while(1) {
        GIC_IntHandler handler;
        u32 irqn = REG_GIC_IRQACK(GIC_THIS_CPU_ALIAS);
        if (irqn == GIC_IRQ_SPURIOUS)
            break;

        handler = GIC_GetCB(irqn, cpu);
        if (handler != NULL)
            handler();

        REG_GIC_IRQEND(GIC_THIS_CPU_ALIAS) = irqn;
    }
}

void GIC_GlobalReset(void)
{
    u32 gicn, intn;

    // Number of local controllers
    gicn = ((REG_DIC_TYPE >> 5) & 3) + 1;

    // Number of interrupt lines (up to 224 external + 32 fixed internal)
    intn = ((REG_DIC_TYPE & 7) + 1) << 5;

    if (gicn > MAX_CPU)
        gicn = MAX_CPU;

    // Clear the interrupt table
    for (u32 i = 0; i < countof(IRQ_Handlers); i++)
        IRQ_Handlers[i] = NULL;

    // Disable all MP11 GICs
    for (u32 i = 0; i < gicn; i++)
        REG_GIC_CONTROL(i) = 0;

    // Disable the main DIC
    REG_DIC_CONTROL = 0;

    // Clear all DIC interrupts
    for (u32 i = 1; i < (intn / 32); i++) {
        REG_DIC_CLRENABLE[i] = ~0;
        REG_DIC_CLRPENDING[i] = ~0;
    }

    // Reset all DIC priorities to lowest and clear target processor regs
    for (u32 i = 32; i < intn; i++) {
        REG_DIC_PRIORITY[i] = 0;
        REG_DIC_TARGETPROC[i] = 0;
    }

    // Set all interrupts to rising edge triggered and 1-N model
    for (u32 i = 2; i < (intn / 16); i++)
        REG_DIC_CFGREG[i] = ~0;

    // Enable the main DIC
    REG_DIC_CONTROL = 1;

    for (u32 i = 0; i < gicn; i++) {
        // Compare all priority bits
        REG_GIC_POI(i) = 3;

        // Don't mask any interrupt with low priority
        REG_GIC_PRIOMASK(i) = 0xF0;

        // Enable the MP11 GIC
        REG_GIC_CONTROL(i) = 1;
    }
}

void GIC_LocalReset(void)
{
    u32 irq_s;

    // Clear out local interrupt configuration bits
    REG_DIC_CLRENABLE[0] = ~0;
    REG_DIC_CLRPENDING[0] = ~0;

    for (int i = 0; i < 32; i++) {
        REG_DIC_PRIORITY[i] = 0;
        REG_DIC_TARGETPROC[i] = 0;
    }

    for (int i = 0; i < 2; i++)
        REG_DIC_CFGREG[i] = ~0;

    // Acknowledge until it gets a spurious IRQ
    do {
        irq_s = REG_GIC_PENDING(GIC_THIS_CPU_ALIAS);
        REG_GIC_IRQEND(GIC_THIS_CPU_ALIAS) = irq_s;
    } while(irq_s != GIC_IRQ_SPURIOUS);
}

int GIC_Enable(u32 irqn, u32 cpumask, u32 prio, GIC_IntHandler handler)
{
    if (!IRQN_IS_VALID(irqn))
        return -1;

    GIC_SetCB(irqn, cpumask, handler);

    REG_DIC_CLRPENDING[irqn >> 5] |= BIT(irqn & 0x1F);
    REG_DIC_SETENABLE[irqn >> 5] |= BIT(irqn & 0x1F);
    REG_DIC_PRIORITY[irqn] = prio << 4;
    REG_DIC_TARGETPROC[irqn] = cpumask;
    return 0;
}

int GIC_Disable(u32 irqn, u32 cpumask)
{
    if (irqn >= MAX_IRQ)
        return -1;

    GIC_ClearCB(irqn, cpumask);

    REG_DIC_CLRPENDING[irqn >> 5] |= BIT(irqn & 0x1F);
    REG_DIC_CLRENABLE[irqn >> 5] |= BIT(irqn & 0x1F);
    REG_DIC_TARGETPROC[irqn] = 0;
    return 0;
}

void GIC_TriggerSoftIRQ(u32 irqn, u32 mode, u32 cpumask)
{
    REG_DIC_SOFTINT = (mode << 24) | (cpumask << 16) | irqn;
}
