/****************************************************************************\
* Copyright (C) 2016 Infineon Technologies
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

/**
* \addtogroup royaleCAPI
* @{
*/

#pragma once

#include <DefinitionsCAPI.h>
#include <CameraDeviceCAPI.h>

ROYALE_CAPI_LINKAGE_TOP

ROYALE_CAPI void royale_get_version (unsigned *major, unsigned *minor, unsigned *patch);
ROYALE_CAPI void royale_get_version_with_build (unsigned *major, unsigned *minor, unsigned *patch, unsigned *build);

ROYALE_CAPI_LINKAGE_BOTTOM
/** @}*/
