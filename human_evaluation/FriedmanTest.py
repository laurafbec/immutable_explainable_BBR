import pandas as pd
from scipy.stats import friedmanchisquare

# Load the data
file_path = "Data/Processed_Data_Participants.xlsx"
df = pd.read_excel(file_path)

# Initialize a results list to store the output for each Scenario ID and Question ID
results = []

# Iterate through all Scenario IDs and Question IDs
for scenario_id in df['Scenario ID'].unique():
    for question_id in df['Question ID'].unique():
        # Filter data for the specific Scenario ID and Question ID
        filtered_data = df[(df['Scenario ID'] == scenario_id) & (df['Question ID'] == question_id)]
        
        # Group responses by Criteria
        criteria_groups = filtered_data.groupby('Criteria')['Response'].apply(list)
        
        # Ensure all criteria have the same number of samples for Friedman Test
        min_length = min(len(responses) for responses in criteria_groups)
        if min_length > 1:  # At least two samples are required for the test
            criteria_responses = [responses[:min_length] for responses in criteria_groups]

            # Perform the Friedman Test
            stat, p_value = friedmanchisquare(*criteria_responses)
            
            # Append results to the list
            results.append({
                'Scenario ID': scenario_id,
                'Question ID': question_id,
                'Friedman Test Statistic': stat,
                'p-value': p_value
            })

# Convert results to a DataFrame for better readability
results_df = pd.DataFrame(results)

# Save the results to an Excel file
results_df.to_excel("Data/Friedman.xlsx", index=False)

print("Friedman test completed. Results saved to 'Friedman.xlsx'.")
