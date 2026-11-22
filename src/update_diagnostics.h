#ifndef CISTERN_UPDATE_DIAGNOSTICS_H
#define CISTERN_UPDATE_DIAGNOSTICS_H

#include <canopennode.h>

#if defined(CONFIG_CISTERN_UPDATE_DIAGNOSTICS)
void cistern_update_diagnostics_attach(CO_SDO_t *sdo);
#endif

#endif
