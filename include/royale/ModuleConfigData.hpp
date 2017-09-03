/****************************************************************************\
* Copyright (C) 2016 Infineon Technologies
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#pragma once

#include <config/ModuleConfig.hpp>

namespace royale
{
    namespace config
    {
        namespace moduleconfig
        {
            /**
            * Configuration for the PicoS with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig PicoS;

            /**
            * Configuration for the PicoFlexx with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig PicoFlexxU6;

            /**
            * Configuration for the Mars module with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig Mars01;

            /**
            * Configuration for the EvalBoard with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig EvalBoard;

            /**
            * Configuration for the EvalBoard
            * "Infineon IRS16x5C Evaluation Kit LED"
            * with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig EvalBoardIRS16x5CLED;

            /**
            * Configuration for the C2 camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig C2Uvc;

            /**
            * Configuration for the CX3 bring-up board with UVC firmware.
            *
            * This is a bring-up board, expected to be used by hardware developers to test new
            * modules, and as a template for the ModuleConfigs for those modules.
            */
            extern const royale::config::ModuleConfig CX3Uvc;

            /**
            * Configuration for the MiniCam.
            */
            extern const royale::config::ModuleConfig MiniCam;

            /**
            * Configuration for the PMDPlatform.
            */
            extern const royale::config::ModuleConfig PMDPlatform;

            /**
            * Configuration for the Skylla camera prototype module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Skylla;

            /**
            * Configuration for the Charybdis camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Charybdis;

            /**
            * Configuration for G8 modules
            */
            extern const royale::config::ModuleConfig G8_FF03;
        }

        // The above aren't usable outside of the DLL they are defined in,
        // so we also provide factory methods:
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigPicoS();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigPicoFlexxU6();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigMars01();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigEvalBoard();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigEvalBoardIRS16x5CLED();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigC2Uvc();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigCX3Uvc();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigMiniCam();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigPMDPlatform();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigSkylla();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigCharybdis();
        ROYALE_API std::unique_ptr<royale::config::ModuleConfig> getModuleConfigG8_FF03();
    }
}
