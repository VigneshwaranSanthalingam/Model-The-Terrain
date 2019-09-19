import bge


def main():

    cont = bge.logic.getCurrentController()
    own = cont.owner

    startgame = cont.actuators["Scene"]
    
    timeAmount = (3 - own["timer"])
    time = round(timeAmount,0)
    own.color=[.365,.6,0,1]
    
    if time <= 0:
        cont.activate(startgame) 
    
    own.text = str(time)
    
    
        
main()
