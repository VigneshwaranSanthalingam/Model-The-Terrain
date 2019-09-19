import bge
import mathutils

def B_controls():

    cont = bge.logic.getCurrentController()
    own = cont.owner
    print(cont)
    front = cont.sensors["W"]
    left = cont.sensors["A"]
    right = cont.sensors["D"]
    back = cont.sensors["S"]
    
    
    if front.positive:
        own.localLinearVelocity.y = 2
    if back.positive:
        own.localLinearVelocity.y = -2
    if left.positive:
        own.applyRotation([0,0,0.05],True)
    if right.positive:
        own.applyRotation([0,0,-0.05],True)
    
    end = cont.sensors["END_A"]
    scene = cont.actuators["SceneA"]
   
    if end.positive:
        own.endObject()
        own.sendMessage("B!!","","")
        cont.activate(scene)
   

        
B_controls()

