/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_BEESIGN)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_vtx_beesign.h"

#include "drivers/beesign.h"
#include "drivers/vtx_common.h"

#include "config/config.h"

#include "io/vtx_beesign.h"
#include "io/vtx.h"

static uint8_t bs_vtxBand;
static uint8_t bs_vtxChannel;
static uint16_t bs_vtxFreq;
static uint16_t bs_showFreq;
static uint8_t bs_vtxPower;
static uint8_t bs_vtxmode;

static uint8_t porModeStr[] = {"  POR MODE"};
static uint8_t porModeFREQStr[] = {"      5584"};

static OSD_TAB_t bsEntryVtxMode = {&bs_vtxmode, BEESIGN_VTX_MODE_COUNT - 1, &bsModeNames[0]};
static OSD_TAB_t bsEntryVtxBand;
static OSD_TAB_t bsEntryVtxChannel;
static OSD_UINT16_t bsEntryVtxFreq = {&bs_vtxFreq, BEESIGN_MIN_FREQUENCY_MHZ, BEESIGN_MAX_FREQUENCY_MHZ, 1};
static OSD_UINT16_t bsShowVtxFreq = {&bs_showFreq, BEESIGN_MIN_FREQUENCY_MHZ, BEESIGN_MAX_FREQUENCY_MHZ, 0};
static OSD_TAB_t bsEntryVtxPower;

static const void *bsCmsConfigMode(displayPort_t *pDisp, const OSD_Entry *self);
static const void *bs_Vtx_onEnter(displayPort_t *pDisp);

static void bs_Vtx_ConfigRead(void)
{
    bs_vtxChannel = vtxSettingsConfig()->channel;
    if (bs_vtxmode == 0) {
        if (vtxSettingsConfig()->band == 0) {
            bs_vtxBand = 0;
        } else {
            bs_vtxBand = vtxSettingsConfig()->band;
        }
        if (vtxSettingsConfig()->freq == 0) {
            vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxCommonDevice(), bs_vtxBand, bs_vtxChannel);
        }
        bs_showFreq = vtxSettingsConfig()->freq;
        bs_vtxFreq = vtxSettingsConfig()->freq;
    } else {
        bs_vtxBand = vtxSettingsConfig()->band;
        bs_vtxFreq = vtxSettingsConfig()->freq;
    }
    bs_vtxPower = vtxSettingsConfig()->power;
}

static const void *bsCmsConfigBandByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (bs_vtxBand == 0) {
        bs_vtxBand = 1;
    }
    if (bs_vtxChannel == 0) {
        bs_vtxChannel = 1;
    }
    if (bs_vtxPower == 0) {
        bs_vtxPower = 1;
    }
    bs_showFreq = vtxCommonLookupFrequency(vtxCommonDevice(), bs_vtxBand, bs_vtxChannel);

    return NULL;
}

static const void *bsCmsConfigRaceSave(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bsSetVtxMode(BEESIGN_VTX_RACE_MODE);
    vtxSettingsConfigMutable()->band = bs_vtxBand;
    vtxSettingsConfigMutable()->channel = bs_vtxChannel;
    vtxSettingsConfigMutable()->power = bs_vtxPower;
    vtxSettingsConfigMutable()->freq = bs_showFreq;

    saveConfigAndNotify();
    return NULL;
}

static const void *bsCmsConfigManualSave(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bsSetVtxMode(BEESIGN_VTX_MANUAL_MODE);
    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = bs_vtxChannel;
    vtxSettingsConfigMutable()->power = bs_vtxPower;
    vtxSettingsConfigMutable()->freq = bs_vtxFreq;
    saveConfigAndNotify();

    return NULL;
}

static const void *bsCmsConfigPorSave(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bsSetVtxMode(BEESIGN_VTX_POR_MODE);
    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = bs_vtxChannel;
    vtxSettingsConfigMutable()->power = (uint8_t)BEESIGN_VTX_PWR_PIT;
    vtxSettingsConfigMutable()->freq = BEESIGN_POR_FREQUENCY_MHZ;
    saveConfigAndNotify();

    return NULL;
}

static const void *bs_Vtx_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    bs_vtxmode = bsDevice.mode;
    bs_Vtx_ConfigRead();

    bsEntryVtxBand.val = &bs_vtxBand;
    bsEntryVtxBand.max = vtxTableBandCount;
    bsEntryVtxBand.names = vtxTableBandNames;

    bsEntryVtxChannel.val = &bs_vtxChannel;
    bsEntryVtxChannel.max = vtxTableChannelCount;
    bsEntryVtxChannel.names = vtxTableChannelNames;

    bsEntryVtxPower.val = &bs_vtxPower;
    bsEntryVtxPower.max = vtxTablePowerLevels;
    bsEntryVtxPower.names = vtxTablePowerLabels;

    return NULL;
}

static OSD_Entry bsCmsMenuModeEntries[] = {
    {"--- BEESIGN MODE ---", OME_Label, NULL, NULL },
    {"MODE",                 OME_TAB,   NULL, &bsEntryVtxMode },
    {"BACK",                 OME_Back,  NULL, NULL },
    {NULL,                   OME_END,   NULL, NULL }
};

CMS_Menu cmsx_menuVtxBsMode = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "VTXBS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = bs_Vtx_onEnter,
    .onExit = bsCmsConfigMode,
    .entries = bsCmsMenuModeEntries
};

static OSD_Entry bsCmsMenuRaceModeEntries[] = {
    {"--- BEESIGN RACE---", OME_Label,            NULL,                  NULL },
    {"BAND",                OME_TAB,              bsCmsConfigBandByGvar, &bsEntryVtxBand },
    {"CHANNEL",             OME_TAB,              bsCmsConfigBandByGvar, &bsEntryVtxChannel },
    {"POWER",               OME_TAB,              NULL,                  &bsEntryVtxPower },
    {"(FREQ)",              OME_UINT16 | DYNAMIC, bsCmsConfigBandByGvar, &bsShowVtxFreq },
    {"SAVE",                OME_Funcall,          bsCmsConfigRaceSave,   NULL },
    {"BACK",                OME_Back,             NULL,                  NULL },
    {NULL,                  OME_END,              NULL,                  NULL }
};

static OSD_Entry bsCmsMenuManualModeEntries[] = {
    {"--- BEESIGN MAMUAL---", OME_Label,   NULL,                  NULL },
    {"MODE",                  OME_Submenu, cmsMenuChange,         &cmsx_menuVtxBsMode },
    {"FREQ",                  OME_UINT16,  NULL,                  &bsEntryVtxFreq },
    {"POWER",                 OME_TAB,     NULL,                  &bsEntryVtxPower },
    {"SAVE",                  OME_Funcall, bsCmsConfigManualSave, NULL },
    {"BACK",                  OME_Back,    NULL,                  NULL },
    {NULL,                    OME_END,     NULL,                  NULL }
};

static OSD_Entry bsCmsMenuPorModeEntries[] = {
    {"--- BEESIGN POR---",    OME_Label,   NULL,                  NULL },
    {"MODE",                  OME_Submenu, cmsMenuChange,         &cmsx_menuVtxBsMode },
    {"(FREQ)",                OME_String,  NULL,                  &porModeFREQStr },
    {"(POWER)",               OME_String,  NULL,                  &porModeStr },
    {"SAVE",                  OME_Funcall, bsCmsConfigPorSave,    NULL },
    {"BACK",                  OME_Back,    NULL,                  NULL },
    {NULL,                    OME_END,     NULL,                  NULL }
};

CMS_Menu cmsx_menuVtxBeesign = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "VTXBS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = bs_Vtx_onEnter,
    .onExit = NULL,
    .entries = bsCmsMenuRaceModeEntries
};

static const void *bsCmsConfigMode(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bs_Vtx_ConfigRead();
    switch (bs_vtxmode) {
        case 0:
            cmsx_menuVtxBeesign.entries = bsCmsMenuRaceModeEntries;
            break;
        case 1:
            cmsx_menuVtxBeesign.entries = bsCmsMenuManualModeEntries;
            break;
        case 2:
            cmsx_menuVtxBeesign.entries = bsCmsMenuPorModeEntries;
            break;
        default:
            cmsx_menuVtxBeesign.entries = bsCmsMenuRaceModeEntries;
            break;
    }
    return NULL;
}
#endif
