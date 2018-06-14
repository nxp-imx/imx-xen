/*==========================================================================*/
/*!
 * @file sci.h
 *
 * Header file containing the public System Controller Interface (SCI)
 * definitions.
 *
 *
 * @{
 */
/*==========================================================================*/

#ifndef _SC_SCI_H
#define _SC_SCI_H

/* Defines */

#define SC_NUM_IPC		5

/* Includes */

#include <xen/types.h>

#include <svc/irq/irq_api.h>
#include <svc/pad/pad_api.h>
#include <svc/pm/pm_api.h>
#include <svc/rm/rm_api.h>
#include <svc/timer/timer_api.h>
#include <svc/misc/misc_api.h>

#include <asm/mx8_mu.h>

/* Types */

/* Functions */
/*!
 * This function initializes the MU connection to SCU.
 *
 * @return  Returns an error code.
 */
int imx8_mu_init(void);

extern sc_ipc_t mu_ipcHandle;
#endif /* _SC_SCI_H */

/**@}*/

