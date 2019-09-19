import bge
import mathutils

def A_controls():

    cont = bge.logic.getCurrentController()
    own = cont.owner
    
    front = cont.sensors["Up Arrow"]
    left = cont.sensors["Left Arrow"]
    right = cont.sensors["Right Arrow"]
    back = cont.sensors["Down Arrow"]
    
    
    if front.positive:
        own.localLinearVelocity.y = 2
    if back.positive:
        own.localLinearVelocity.y = -2
    if left.positive:
        own.applyRotation([0,0,0.05],True)
    if right.positive:
        own.applyRotation([0,0,-0.05],True)
        
     
    end = cont.sensors["END_B"]
    scene = cont.actuators["SceneB"]
    
    if end.positive:
        own.endObject()
        own.sendMessage("A!!","","")
        cont.activate(scene)
   
        
A_controls()
