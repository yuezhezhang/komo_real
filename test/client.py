import zerorpc

client = zerorpc.Client()
client.connect("tcp://127.0.0.1:4242")

q0 = [1, 1, 1]
q1 = [2, 2, 2]
while True:
    input_data = "hello from client"
    result = client.process_data(input_data)
    obs_pos = client.get_obs_pos()
    print(f"Sent: {input_data}")
    print(f"Received: {result}")
    print(f"Obs data:, {obs_pos}")
    # update C

    # run KOMO/RRT, plan to goal

    # if reach q1, goal = q0

    # if reach q0, goal = q1