import numpy as np
import matplotlib.pyplot as plt
import json
def load_data_from_file(filename):
    data = []
    try:
        with open(filename, 'r') as file:
            for line in file:
                data.append(json.loads(line.strip()))
    except Exception as e:
        print(f"Error reading data from file: {e}")
    return data
def transform_data(data):
    transformed_data = {
        "target_status": [],
        "movement_distance": [],
        "movement_energy": [],
        "stationary_distance": [],
        "stationary_energy": [],
        "detection_distance": []
    }

    for entry in data:
        transformed_data["target_status"].append(entry["target_status"])
        transformed_data["movement_distance"].append(entry["movement_distance"])
        transformed_data["movement_energy"].append(entry["movement_energy"])
        transformed_data["stationary_distance"].append(entry["stationary_distance"])
        transformed_data["stationary_energy"].append(entry["stationary_energy"])
        transformed_data["detection_distance"].append(entry["detection_distance"])

    return transformed_data
if __name__ == "__main__":
    data = load_data_from_file("sensor_data.json")
    transformed_data = transform_data(data)
    print(data)
#%%

# Convert dictionary values to NumPy arrays
target_status = np.array(transformed_data["target_status"])
movement_distance = np.array(transformed_data["movement_distance"])
movement_energy = np.array(transformed_data["movement_energy"])
stationary_distance = np.array(transformed_data["stationary_distance"])
stationary_energy = np.array(transformed_data["stationary_energy"])
detection_distance = np.array(transformed_data["detection_distance"])

filtered_movement_distance = movement_distance[movement_energy >= 90]

# Plot the filtered data
plt.plot(filtered_movement_distance)
plt.xlabel('Index')
plt.ylabel('Movement Distance')
plt.title('Movement Distance for Movement Energy >= 90')
plt.show()
#%%

