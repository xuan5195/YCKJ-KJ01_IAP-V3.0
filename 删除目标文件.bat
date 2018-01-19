rd /Q /S project\MDK-ARM(uV3)\Flash
rd /Q /S project\MDK-ARM(uV3)\CpuRAM
rd /Q /S project\MDK-ARM(uV3)\ExtSRAM
del /Q project\MDK-ARM(uV3)\*.bak
del /Q project\MDK-ARM(uV3)\*.dep
del /Q project\MDK-ARM(uV3)\JLink*

rd /Q /S project\MDK-ARM(uV4)\Flash
rd /Q /S project\MDK-ARM(uV4)\CpuRAM
rd /Q /S project\MDK-ARM(uV4)\ExtSRAM
del /Q project\MDK-ARM(uV4)\*.bak
del /Q project\MDK-ARM(uV4)\*.dep
del /Q project\MDK-ARM(uV4)\JLink*
del /Q project\MDK-ARM(uV4)\project.uvgui.*

rd /Q /S RVMDK\Flash
rd /Q /S RVMDK\CpuRAM
rd /Q /S RVMDK\ExtSRAM
del /Q RVMDK\*.bak
del /Q RVMDK\*.dep
del /Q RVMDK\JLink*
del /Q RVMDK\project.uvgui.*

del /Q project\EWARMv5\Project.dep
del /Q project\EWARMv5\Flash
del /Q project\EWARMv5\CpuRAM
del /Q project\EWARMv5\settings
rd  /Q /S project\EWARMv5\Flash
rd /Q /S project\EWARMv5\CpuRAM
rd /Q /S project\EWARMv5\ExtSRAM
rd /Q /S project\EWARMv5\settings

