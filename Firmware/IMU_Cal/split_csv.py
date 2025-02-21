
import pandas as pd

# Read the original CSV file
input_file = 'C:\\Users\\dougl\\OneDrive\\Documents\\year 5\\Project\\Event_Based_Navigation\\Firmware\\IMU_Cal\\raw.csv'  # Replace with your actual file path
try:
    df = pd.read_csv(input_file)
    print("CSV file loaded successfully.")
except Exception as e:
    print(f"Error reading the CSV file: {e}")
    exit()

# Check the first few rows of the dataframe to make sure it's being read correctly
print("First few rows of the dataset:")
print(df.head())

# Check if the CSV file has the expected number of columns (at least 6)
if df.shape[1] < 6:
    print("Warning: The CSV file doesn't have enough columns (at least 6).")
else:
    # Split the dataframe into accelerometer and magnetometer data
    accel_data = df.iloc[:, 0:3]  # Assuming columns 1, 2, 3 are accelerometer values
    mag_data = df.iloc[:, 3:6]    # Assuming columns 4, 5, 6 are magnetometer values

    # Save the separate dataframes into new CSV files
    accel_data.to_csv('C:\\Users\\dougl\\OneDrive\\Documents\\year 5\\Project\\Event_Based_Navigation\\Firmware\\IMU_Cal\\accelerometer_data.csv', index=False)
    mag_data.to_csv('C:\\Users\\dougl\\OneDrive\\Documents\\year 5\\Project\\Event_Based_Navigation\\Firmware\\IMU_Cal\\magnetometer_data.csv', index=False)

    print("Data has been successfully split into 'accelerometer_data.csv' and 'magnetometer_data.csv'.")

