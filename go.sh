clear
date
g++ `pkg-config --libs opencv` main.cpp ILI9341.cpp -o camera -l bcm2835 
date