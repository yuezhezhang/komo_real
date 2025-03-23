import zerorpc

class DataServer:
    def process_data(self, input_data):
        return f"Processed: {input_data.upper()}"
    def get_obs_pos(self):
        return [1, 2, 3]

server = zerorpc.Server(DataServer())
server.bind("tcp://0.0.0.0:4242")
print("Server is running...")
server.run()