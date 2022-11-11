import time


def accurate_delay(delay):
    '''Function to provide accurate time delay in millisecond'''
    _ = time.perf_counter() + delay / 1000
    while time.perf_counter() < _:
        pass


start = time.time()

for i in range(1000000):
    a = i**2


# accurate_delay(1000 - (time.time() - start) * 1000)
time.sleep(1 - (time.time() - start))

print(time.time() - start)
