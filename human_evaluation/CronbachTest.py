import pandas as pd
import numpy as np

# Load the dataset
file_path = 'Data/Processed_Data.xlsx'
data = pd.ExcelFile(file_path).parse('Sheet1')

# Step 1: Transform the dataset (Pivot Criteria values into columns)
aggregated_data = data.groupby(
    ["Scenario ID", "Question ID", "Category ID", "Criteria"], as_index=False
).mean()

transformed_data = aggregated_data.pivot_table(
    index=["Scenario ID", "Question ID", "Category ID"],
    columns="Criteria",
    values="Response"
).reset_index()

transformed_data.columns.name = None

# Export the transformed data to Excel
output_excel_path = 'Transformed_Data.xlsx'
transformed_data.to_excel(output_excel_path, index=False, sheet_name="Transformed Data")

# Select the criteria columns
criteria_columns = transformed_data.columns.difference(["Scenario ID", "Question ID", "Category ID"])
item_scores = transformed_data[criteria_columns]

# Handle missing values by filling with the mean of each column
item_scores = item_scores.fillna(item_scores.mean())

# Step 2: Calculate Cronbach's Alpha (Overall)
k = item_scores.shape[1]  # Number of items
item_variances = item_scores.var(axis=0, ddof=1)  # Variance of each item
total_scores = item_scores.sum(axis=1)  # Total scores across items
total_variance = total_scores.var(ddof=1)  # Variance of total scores

# Cronbach's Alpha calculation
cronbach_alpha = (k / (k - 1)) * (1 - (item_variances.sum() / total_variance))
print(f"Cronbach's Alpha (Overall): {cronbach_alpha:.4f}")

# Step 3: Calculate Cronbach's Alpha if an item is deleted
alpha_if_deleted = {}
for criterion in criteria_columns:
    # Subset data excluding the current item
    subset = item_scores.drop(columns=[criterion])
    k_subset = subset.shape[1]
    item_variances_subset = subset.var(axis=0, ddof=1)
    total_scores_subset = subset.sum(axis=1)
    total_variance_subset = total_scores_subset.var(ddof=1)

    # Recalculate Cronbach's Alpha
    alpha_subset = (k_subset / (k_subset - 1)) * (1 - (item_variances_subset.sum() / total_variance_subset))
    alpha_if_deleted[criterion] = alpha_subset

# Convert to DataFrame
alpha_if_deleted_df = pd.DataFrame.from_dict(alpha_if_deleted, orient='index', columns=["Alpha if Deleted"])
alpha_if_deleted_df.reset_index(inplace=True)
alpha_if_deleted_df.rename(columns={'index': 'Criterion'}, inplace=True)

# Step 4: Calculate Item-Total Correlations
item_total_correlation = {}
for criterion in criteria_columns:
    # Correlation of each item with the total score excluding that item
    total_excluding_item = total_scores - item_scores[criterion]
    correlation = item_scores[criterion].corr(total_excluding_item)
    item_total_correlation[criterion] = correlation

# Convert to DataFrame
item_total_corr_df = pd.DataFrame.from_dict(item_total_correlation, orient='index', columns=["Item-Total Correlation"])
item_total_corr_df.reset_index(inplace=True)
item_total_corr_df.rename(columns={'index': 'Criterion'}, inplace=True)

# Step 5: Calculate Inter-Item Correlations
inter_item_corr = item_scores.corr()

# Step 6: Display and Export Results
print("\nCronbach's Alpha if Item Deleted:")
print(alpha_if_deleted_df)

print("\nItem-Total Correlations:")
print(item_total_corr_df)

print("\nInter-Item Correlations:")
print(inter_item_corr)

# Export all results to an Excel file
with pd.ExcelWriter('Data/Reliability_Analysis_Results.xlsx') as writer:
    alpha_if_deleted_df.to_excel(writer, index=False, sheet_name="Alpha if Deleted")
    item_total_corr_df.to_excel(writer, index=False, sheet_name="Item-Total Correlation")
    inter_item_corr.to_excel(writer, sheet_name="Inter-Item Correlations")
