import pandas as pd

# File and sheet names
file_path = 'Data\TotalQA.xlsx'
sheet_names = ['Scn1', 'Scn2', 'Scn3']

# Subquestion to Category mapping
subquestion_to_category = {
    1: 1, 2: 1, 3: 2, 4: 2, 5: 3, 6: 3, 7: 3, 8: 4,
    9: 4, 10: 5, 11: 5, 12: 4, 13: 2, 14: 4, 15: 3, 16: 5
}

# Criteria mapping based on the last digit of Question ID
criteria_mapping = {
    '1': 'Understability',
    '2': 'Satisfaction',
    '3': 'Informativeness',
    '4': 'Completeness',
    '5': 'Usefulness'
}

# Initialize a list to store processed data
all_data = []

# Process each sheet
for i, sheet_name in enumerate(sheet_names, start=1):
    # Load the sheet
    df = pd.read_excel(file_path, sheet_name=sheet_name)

    # Assign Scenario ID
    df['Scenario ID'] = i

    # Reshape from wide to long format
    long_df = df.melt(id_vars=['Scenario ID'],
                      var_name='Question ID',
                      value_name='Response')

    # Extract Subquestion ID as the first part of Question ID (before the dot)
    long_df['Subquestion ID'] = long_df['Question ID'].str.split('.').str[0].astype(int)

    # Map Subquestion ID to Category ID
    long_df['Category ID'] = long_df['Subquestion ID'].map(subquestion_to_category)

    # Extract the last digit of Question ID (after the dot) to determine the criterion
    long_df['Criteria'] = long_df['Question ID'].str.split('.').str[1].map(criteria_mapping)

    # Append processed data
    all_data.append(long_df)

# Concatenate all processed data into a single DataFrame
final_data = pd.concat(all_data, ignore_index=True)

# Save the consolidated data to a new Excel file
output_file = 'Processed_Data.xlsx'
final_data.to_excel(output_file, index=False)

print(f"Processed data saved to {output_file}")
