import matplotlib.pyplot as plt
import re
import numpy as np
import math

# Get test file
file = open("TestingLocations.txt")
test_data = file.read()
file.close()

# Get log file
file = open("full_log.txt")
log_data = file.read()
file.close()

# Regular expression patterns
patternTests = re.compile(r"\((\d+), (\d+)\)")
patternLogs = re.compile(r"CM: Button Pressed at pose: \((-?\d+),(-?\d+),-?\d+\)")

# Extract log points
log_points = patternLogs.findall(log_data)
log_points = [(int(x), int(y)) for x, y in log_points]
log_index = 0

# Split the test data into individual test sections
test_sections = test_data.split("Test ")[1:]

# Iterate through each test section
for test in test_sections:
    lines = test.strip().split("\n")
    test_name = lines[0].strip()
    print("\n"+test_name)

    points = patternTests.findall(test)
    points = [(int(x), int(y)) for x, y in points]

    # Get the corresponding log points for this test
    test_log_points = log_points[log_index:log_index + len(points)]
    log_index += len(points)

    # Extract x and y values for plotting
    test_x, test_y = zip(*points)
    log_x, log_y = zip(*test_log_points)

    log_corrected_x = np.array(log_x) - log_x[0]
    log_corrected_y = np.array(log_y) - log_y[0]

    plt.figure(figsize=(8, 8))

    # Plot dotted red error lines between corresponding test and log points
    i = 0;
    for tx, ty, lx, ly in zip(test_x, test_y, log_corrected_x, log_corrected_y):
        plt.plot([tx, lx], [ty, ly], 'r:', label='Error Line' if tx == test_x[0] else "")
        i += 1
        print(f"Error Point {i} = {math.sqrt((lx-tx)^2+(ly-ty)^2)}")

    plt.scatter(test_x, test_y, marker='o', label=f'Test {test_name} Points', color='b')
    plt.scatter(log_x, log_y, marker='x', color='k', label=f'Log Data {test_name}')
    plt.scatter(log_corrected_x, log_corrected_y, marker='x', color='g', label=f'Log Corrected Data {test_name}')

    # Labels and title
    plt.xlabel('X Position (cm)')
    plt.ylabel('Y Position (cm)')
    plt.title(f'Test {test_name} vs Log Data')
    plt.legend()
    plt.grid(True)
    plt.show()
