import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import csv

def sign(num):
    if num > 0:
        return 1
    return -1

# Finds time of Deadband Starts and Ends with Corresponding Inputs #
def find_constants(path, constants_path):
    # Readiong in Data into the DataFrame #
    df = pd.read_csv(path)
    df['Relative time'] = df['time'] - df['time'].iloc[0]

    # Calculating Slope Ends where the input takes Maximum Value and finding Corresponing time #
    slope_ends = {'positive':df.iloc[df['input'].idxmax()]['Relative time'],
                'negative':df.iloc[df['input'].idxmin()]['Relative time']}               

    # Empty Lists for Deadband Starts, Ends and Coefficients #
    deadband_starts = []
    deadband_ends = []
    kinetic_coeffs = []
    static_coeffs = []

    # Delay to prevent incorrect measurement of static coefficient #

    # Iterate over Each Measured Instant #
    for i in range(len(df)):
        # Extracting row for current time instant #
        ser = df.iloc[i]

        # If wheel speed is not turning, and was previously turning, potentially a kinetic coefficient #
        if ser['velocity'] == 0 and df.iloc[i-1]['velocity'] != 0 and df.iloc[i-2]['velocity'] != 0  and df.iloc[i-3]['velocity'] != 0 and df.iloc[i-4]['velocity'] != 0 :
            # To Prevent Indexing Error with kinetic_coeff #
            if len(kinetic_coeffs) == 0:
                deadband_starts.append(ser['Relative time'])
                kinetic_coeffs.append(ser['input'])
            # If the current instant is suspected to be the kinetic coefficient, it must not have the same sign as the previous kinetic coefficient #
            elif sign(kinetic_coeffs[-1]) != sign(ser['input']):
                deadband_starts.append(ser['Relative time'])
                kinetic_coeffs.append(ser['input'])
        
        # If the wheel is about to start turning, and was not turning, potentially a static coefficient #
        if ser['velocity'] == 0 and df.iloc[i-1]['velocity'] == 0 and df.iloc[i+1]['velocity'] != 0 and df.iloc[i+2]['velocity'] != 0 and  df.iloc[i+3]['velocity'] != 0 :
            # To Prevent Indexing Error with static_coeffs #
            if len(static_coeffs) == 0:
                deadband_ends.append(ser['Relative time'])
                static_coeffs.append(ser['input'])
            # For the static deadband we want the last point before which the wheel starts continuously turning, until then replace the last suspected point #
            elif sign(static_coeffs[-1]) == sign(ser['input']):
                deadband_ends[-1] = ser['Relative time']
                static_coeffs[-1] = ser['input']
            # Append the point to the static coefficients if the sign has changed #
            else:
                deadband_ends.append(ser['Relative time'])
                static_coeffs.append(ser['input'])

    # Finding Kinetic Coefficients in each direction by averaging #
    deadband_starts_dec = []
    deadband_starts_inc = []

    deadband_ends_inc = []
    deadband_ends_dec = []
    
    kinetic_dec = []
    kinetic_inc = []
    
    static_inc = []
    neg_static = []

    for i in range(len(kinetic_coeffs)):
        if kinetic_coeffs[i] > 0:
            kinetic_dec.append(kinetic_coeffs[i])
            deadband_starts_dec.append(deadband_starts[i])
        else:
            kinetic_inc.append(kinetic_coeffs[i])
            deadband_starts_inc.append(deadband_starts[i])
    
    for i in range(len(static_coeffs)):
        if static_coeffs[i] > 0:
            static_inc.append(static_coeffs[i])
            deadband_ends_inc.append(deadband_ends[i])
        else:
            neg_static.append(static_coeffs[i])
            deadband_ends_dec.append(deadband_ends[i])

    # Print all constants#
    # print(f'{deadband_starts=}')
    # print(f'{deadband_ends=}')
    # print(f'{kinetic_dec=}')
    # print(f'{kinetic_inc=}')
    # print(f'{static_inc=}')
    # print(f'{neg_static=}') 

    data_constants = {
        'deadband_starts_dec': deadband_starts_dec,
        "kinetic_coeffs_dec": kinetic_dec,
        'deadband_starts_inc': deadband_starts_inc,
        "kinetic_coeffs_inc": kinetic_inc, 
        'deadband_ends_dec': deadband_ends_dec,
        "static_coeffs_dec": neg_static,
        'deadband_ends_inc': deadband_ends_inc,
        "static_coeffs_inc": static_inc,
        }

    max_len = max(len(v) for v in data_constants.values())
    df_constants = pd.DataFrame({k: [v[i] if i < len(v) else pd.NA for i in range(max_len)] for k, v in data_constants.items()})
    df_constants.to_csv(constants_path,index=False)

    return slope_ends

# Plots Raw Data with Constants found in find_constants #
def plot_raw_with_measured_constants(path,constants_path):
    df = pd.read_csv(path)
    df['Relative time'] = df['time'] - df['time'].iloc[0]
    deadbands_df = pd.read_csv(constants_path)

    plt.subplot(2,1,1)
    plt.plot(df['Relative time'],df['input'])
    for _,row in deadbands_df.iterrows():
        plt.axvline(x=row['deadband_starts_dec'],color='k',linestyle='--',linewidth=1)
        plt.axvline(x=row['deadband_ends_dec'],color='k',linestyle='--',linewidth=1)
        plt.axvline(x=row['deadband_starts_inc'],color='k',linestyle='--',linewidth=1)
        plt.axvline(x=row['deadband_ends_inc'],color='k',linestyle='--',linewidth=1)

        plt.plot(row['deadband_starts_dec'],row['kinetic_coeffs_dec'],'o')
        plt.plot(row['deadband_starts_inc'],row['kinetic_coeffs_inc'],'o')
        plt.plot(row['deadband_ends_dec'],row['static_coeffs_dec'],'o')
        plt.plot(row['deadband_ends_inc'],row['static_coeffs_inc'],'o')
    plt.title(path)
    
    plt.subplot(2,1,2)
    plt.plot(df['Relative time'],df['velocity'])
    for _,row in deadbands_df.iterrows():
        plt.axvline(x=row['deadband_starts_dec'],color='k',linestyle='--',linewidth=1)
        plt.axvline(x=row['deadband_ends_dec'],color='k',linestyle='--',linewidth=1)
        plt.axvline(x=row['deadband_starts_inc'],color='k',linestyle='--',linewidth=1)
        plt.axvline(x=row['deadband_ends_inc'],color='k',linestyle='--',linewidth=1)

    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()
    plt.show()

def suggest_compensation(constants_path):
    df_constants = pd.read_csv(constants_path)
    df_constants.dropna(axis=0, how='any', inplace=True)

    # Calculate the mean of each column
    mean_values = df_constants.mean()

    # Print the mean values
    print("Mean values:")
    print(mean_values)

if __name__ == '__main__':
    ## Use to Plot a Particular File ##
    path = "./data/deadband_test_2025-04-21 18:21:50.771442.csv"
    constants_path = "./coeffs/deadband_test_2025-04-21 18:21:50.771442.csv"
    find_constants(path, constants_path)
    plot_raw_with_measured_constants(path,constants_path)
    suggest_compensation(constants_path)
    
    ## Use To Create Slope Constants CSV ##
    # with open(slope_ends_path, "w", newline="\n") as f:
    #     csv.writer(f, delimiter=',').wrwiterow(['speed', 'motor', 'positive', 'negative'])
    
    ## Use to Plot Data of Particular Speeds for Both Motors ##
    # speeds = [1,2,3,4,5,6,7,8,9,10,15,20,25,30,35,40]
    # motors = ['Front']
    # slope_ends_path = "./Python/deadband coeff/plot constants/SlopeEnds10.csv"
    
    # for speed in speeds:
    #     for motor in motors:
    #         path = f'./Python/deadband tests/deadband coeff/SourceData10/{motor}Slope{speed}Data.csv' 
    #         constants_path = f'./Python/deadband tests/deadband coeff/plot constants/CombinedConstants/{motor}Slope{speed}Constants.csv'

    #         plot_raw_with_measured_constants(path,constants_path)
            # slope_ends = find_constants(path, constants_path)

            # with open(slope_ends_path, "a", newline="\n") as f:
            #     csv.writer(f, delimiter=',').writerow([speed, motor, slope_ends['positive'], slope_ends['negative']])

    ## Use to Plot Data of Particular Speeds-Motors Combination ##
    # speeds = [1,5,6,15,25,30,35,40,3,20,25,30,35]
    # motors = ['Front','Front','Front','Front','Front','Front','Front','Front','Rear','Rear','Rear','Rear','Rear']
    # for speed,motor in zip(speeds,motors):
    #     path = f'./Python/deadband tests/deadband coeff/SourceData10/{motor}Slope{speed}Data.csv' 
    #     constants_path = f'./Python/deadband tests/deadband coeff/plot constants/CombinedConstants/{motor}Slope{speed}Constants.csv'

    #     plot_raw_with_measured_constants(path,constants_path)
