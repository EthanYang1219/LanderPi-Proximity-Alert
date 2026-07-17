from gpiozero import TonalBuzzer 
from gpiozero.tones import Tone
from time import sleep

buzzer = TonalBuzzer(17)
buzzer.play("A4") #Produces A4 Note
sleep(1) #Plays the tone for 1 second
buzzer.play("C#5")
sleep(1)
buzzer.play(500) #Produces a 400Hz Tone
sleep(1)
buzzer.play(Tone(440)) 
sleep(1)
buzzer.stop() #Stops the buzzer
