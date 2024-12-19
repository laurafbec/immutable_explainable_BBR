import scikit_posthocs as sp
import pandas as pd

# Load the data
file_path = 'Data/Processed_Data_Participants.xlsx'
df = pd.read_excel(file_path)

# Initialize results
posthoc_results = []

# Iterate over each Scenario ID and Question ID
for scenario_id in df['Scenario ID'].unique():
    for question_id in df['Question ID'].unique():
        # Filter data for the current Scenario ID and Question ID
        filtered_data = df[(df['Scenario ID'] == scenario_id) & (df['Question ID'] == question_id)]

        # Pivot the data into wide format (required by Nemenyi test)
        pivot_data = filtered_data.pivot(index='Participant ID', columns='Criteria', values='Response')
        
        if pivot_data.shape[1] > 1:  # Ensure at least two groups
            # Perform the Nemenyi test
            p_values = sp.posthoc_nemenyi_friedman(pivot_data.T)
            
            # Save results
            posthoc_results.append({
                'Scenario ID': scenario_id,
                'Question ID': question_id,
                'Post-hoc P-Values': p_values
            })

# Example: Access the results for one combination
for result in posthoc_results:
    print(f"Scenario ID: {result['Scenario ID']}, Question ID: {result['Question ID']}")
    print(result['Post-hoc P-Values'])
