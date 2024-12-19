import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats

# Load your data
# Replace 'your_file.xlsx' and 'Sheet1' with the appropriate file and sheet names
file_path = 'Data/ProcessedData.xlsx'
data = pd.read_excel(file_path, sheet_name='Sheet1')

# Function to analyze normality
def analyze_normality(data, group_columns, response_column):
    results = []
    for group, subset in data.groupby(group_columns):
        if len(subset[response_column]) > 3:  # Shapiro-Wilk requires at least 3 data points
            # Shapiro-Wilk Test
            shapiro_stat, shapiro_p = stats.shapiro(subset[response_column])

            # Skewness and Kurtosis
            skewness = stats.skew(subset[response_column])
            kurtosis = stats.kurtosis(subset[response_column])

            # Append results
            results.append({
                'Group': group,
                'Shapiro Stat': shapiro_stat,
                'P-Value': shapiro_p,
                'Skewness': skewness,
                'Kurtosis': kurtosis,
                'Normality': 'Normal' if shapiro_p > 0.05 else 'Not Normal'
            })

            # Visualization: Histogram
            plt.figure(figsize=(8, 6))
            plt.hist(subset[response_column], bins=10, alpha=0.7, edgecolor='black', density=True)
            plt.title(f'Histogram for Group: {group}')
            plt.xlabel(response_column)
            plt.ylabel('Density')
            plt.show()

            # Visualization: Q-Q Plot
            plt.figure(figsize=(8, 6))
            stats.probplot(subset[response_column], dist="norm", plot=plt)
            plt.title(f'Q-Q Plot for Group: {group}')
            plt.show()

    # Convert results to DataFrame for export or further analysis
    results_df = pd.DataFrame(results)
    return results_df

# Usage Example
# Group by 'Scenario ID' and 'Category ID', analyze normality of 'Response'
results_df = analyze_normality(data, group_columns=['Scenario ID', 'Category ID'], response_column='Response')

# Save results to a CSV file
results_df.to_csv('normality_analysis_results.csv', index=False)

# Display results
print(results_df)
