

def get_distance(array):
    time = len(array) * 0.1
    distance = 0
    for i in range(array):
        distance += array[i] * 0.1

    return distance


def get_velocity(array):
    velocity = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")

            except:
                print('')

            print("Current Depth: ", data)

    return velocity
