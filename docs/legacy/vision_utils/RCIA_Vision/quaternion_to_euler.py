import math

T = """
---
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "map"
    ns: "armors"
    id: 0
    type: 1
    action: 0
    pose: 
      position: 
        x: 4.473381319209877
        y: 0.4225171227229736
        z: 0.25
      orientation: 
        x: 0.02398065794544013
        y: 0.12830438379092762
        z: -0.18215118121493196
        w: 0.9745685508606047
    scale: 
      x: 0.2
      y: 1.5
      z: 0.8
    color: 
      r: 0.0
      g: 0.75
      b: 0.949999988079071
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "map"
    ns: "armors"
    id: 1
    type: 1
    action: 0
    pose: 
      position: 
        x: 8.77085657056893
        y: -3.259132906158988
        z: 0.25
      orientation: 
        x: 0.12276727108439343
        y: 0.04433095781832885
        z: -0.9325114014672202
        w: 0.33672755970226353
    scale: 
      x: 0.2
      y: 1.5
      z: 0.8
    color: 
      r: 0.0
      g: 0.75
      b: 0.949999988079071
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "map"
    ns: "chassisCentral"
    id: 5
    type: 3
    action: 0
    pose: 
      position: 
        x: 7.270856570568929
        y: -0.6610566948056726
        z: 0.25
      orientation: 
        x: 0.0
        y: -0.0
        z: -0.18372295657726387
        w: 0.9829780644686374
    scale: 
      x: 0.35
      y: 0.35
      z: 2.5
    color: 
      r: 0.0
      g: 0.949999988079071
      b: 0.3499999940395355
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "map"
    ns: "armors"
    id: 2
    type: 1
    action: 0
    pose: 
      position: 
        x: 8.77085657056893
        y: 1.9370195165476431
        z: 0.25
      orientation: 
        x: -0.09857372908271643
        y: 0.08555849814791414
        z: 0.7487429299587917
        w: 0.6498822879307643
    scale: 
      x: 0.2
      y: 1.5
      z: 0.8
    color: 
      r: 0.0
      g: 0.75
      b: 0.949999988079071
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
---

"""

for line in T.split("\n"):
    if "ns:" in line:
        ns = line.split(": ")[1].replace('"', '')
    elif "id:" in line and "frame" not in line:
        if ns == "armors":
            id = int(line.split(": ")[1]) + 1
        elif ns == "chassisCentral":
            id = 0

    if "x:" in line:
        x = float(line.split(": ")[1])
    elif "y:" in line:
        y = float(line.split(": ")[1])
    elif "z:" in line:
        z = float(line.split(": ")[1])
    elif "w:" in line:
        w = float(line.split(": ")[1])

    if "scale:" in line:
        # print(x, y, z, w)
        print("\n")
        print(f"Type: {ns + str(id)}")
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        print(f"Roll: {round(roll_deg, 3)} degrees")
        print(f"Pitch: {round(pitch_deg, 3)} degrees")
        print(f"Yaw: {round(yaw_deg, 3)} degrees")