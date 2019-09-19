import bge


def main():

    cont = bge.logic.getCurrentController()
    own = cont.owner

    startgame = cont.actuators["Scene"]
    
    timeAmount = (30 - own["timer"])
    time = round(timeAmount,1)
    
    if time <= 0:
        cont.activate("startgame") 
    if time >= 10:
        own.color=[0.0,1.0,0.0,1.0]
    if time <= 20:
        own.color=[1.0,0.3,0.0,1.0] 
    if time <= 10:
        own.color=[1.0,0.0,0.0,1.0]    
    own.text = str(time)
    
    
        
main()
