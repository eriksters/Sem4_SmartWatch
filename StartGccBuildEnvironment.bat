@echo off

rem *************************************************************************************************
rem Please set the ABSOLUTE installation path of the ST-LINK utility
rem *************************************************************************************************

set ST-LINK_Utility_Path=D:\ST_ToolChain\ST-LINK Utility

rem *************************************************************************************************
rem Please do NOT change the following settings
rem *************************************************************************************************
set PlatformName=STM32F469-Discovery
set ExternalLoader=N25Q128A_STM32469I-DISCO.stldr
set BuildEnvVersion=V9.30
set GettingStartedLink=getting-started-stm32f469-discovery
cmd /K Application\Project\GCC\devenv.cmd
