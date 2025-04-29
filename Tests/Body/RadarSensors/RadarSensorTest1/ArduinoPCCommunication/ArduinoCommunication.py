from RUStack import * # importeer de RUStack library
import time # importeer de library voor datum- en tijdfuncties

mystack=RUStack() # maak een RUStack object om te communiceren met de M5Stack
mystack.autoconnect() # maak verbinding met de M5Stack
mystack.display.turn_off() # zet het display van de M5Stack uit

file = open("testfile.txt","w") # maak een nieuw bestand en open het met schrijfpermissie

starttime=time.time_ns()//1000 # De time_ns() method geeft het aantal nanoseconden dat, op het moment dat de functie wordt aangeroepen, verstreken is sinds een bepaald afgesproken tijdstip, meestal (1 januari, 1970, 00:00:00 (UTC)). We bewaren hier de begintijd, deze kunnen we dan later steeds van de opgevraagde tijd aftrekken om een relatieve tijdmeting te krijgen ten opzichte van deze eerste aanroep. We delen door 1000 om de tijd in microseconden te krijgen.

try:
    while True: # blijf de hele tijd bezig met het volgende:
        file.write(str(time.time_ns()//1000-starttime)) # schrijf (de huidige tijd - de begintijd) naar het bestand als timestamp
        file.write(",") # gevolgd door een komma
        file.write(str(mystack.mpu6886.accX)) # en de meting van de x-component van de accelerometer
        file.write("\n") # en ga naar een nieuwe regel

except KeyboardInterrupt: # stop als er op een toets wordt gedrukt (werkt niet onder Pyzo, daarin stoppen met ctrl-K)
    pass

mystack.display.turn_on() # zet het display weer aan
mystack.disconnect() # verbreek de verbinding met de M5Stack
file.close() # sluit het bestand