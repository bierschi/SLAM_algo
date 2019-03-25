echo "upload to device STM32...\n"
cp ./STM32_HSPCarBin ./../TOOLS/stlink-master/STM32_HSPCarBin
cd ./../TOOLS/stlink-master
sudo ./st-flash write ./STM32_HSPCarBin 0x8000000
echo "\nuploading done!"

