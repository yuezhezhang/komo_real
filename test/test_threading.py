import threading
import time

class SharedDataExample:
    def __init__(self):
        # Shared data
        self.shared_data = []
        # Lock to ensure thread safety
        self.lock = threading.Lock()

    def stream_data(self):
        i = 0
        while True:
            i += 1
            # Lock before modifying the shared data
            with self.lock:
                # Add data to the shared resource
                self.shared_data.append(i)
                print("Function1 added data ", i)

            # Simulate some work
            time.sleep(1)

    def process_data(self):
        while True:
            # Lock before accessing the shared data
            with self.lock:
                if self.shared_data:
                    # Process and remove data from the shared resource
                    data = self.shared_data.pop(0)
                    print(f"Function2 processed: {data}")

            # Simulate some work
            time.sleep(1)

    def start_threads(self):
        # Create threads for both functions
        thread1 = threading.Thread(target=self.stream_data)
        thread2 = threading.Thread(target=self.process_data)

        # Start the threads
        thread1.start()
        thread2.start()

        # Optionally, join threads (blocks the main thread)
        thread1.join()
        thread2.join()


# Create an instance of the class and start the threads
example = SharedDataExample()
example.start_threads()
