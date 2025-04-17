@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\Users\matth\Downloads\Combine_Ver12_coin_speaker_UNTESTED\"
"C:\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\Users\matth\Downloads\Combine_Ver12_coin_speaker_UNTESTED\JDY40_test.c"
if not exist hex2mif.exe goto done
if exist JDY40_test.ihx hex2mif JDY40_test.ihx
if exist JDY40_test.hex hex2mif JDY40_test.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\Users\matth\Downloads\Combine_Ver12_coin_speaker_UNTESTED\JDY40_test.hex
