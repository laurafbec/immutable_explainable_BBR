import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize

# Load data from the Excel file
file_path = "Data\Processed_Data.xlsx"  # Replace with the actual path to your Excel file
sheet_name = "Sheet1"  # Replace with the sheet name if necessary

# Read the Excel file into a DataFrame
df = pd.read_excel(file_path, sheet_name=sheet_name)

# Ensure data is loaded correctly
print("Data Preview:")
print(df.head())  # Preview the first few rows to check the data

# Calculate the mean response per Scenario ID and Category ID
correlation_data = df.groupby(['Scenario ID', 'Category ID'])['Response'].mean().unstack()

# Calculate the correlation matrix
correlation_matrix = correlation_data.corr()

# Define a custom palette with adjusted red tones for strong correlations
graphic_colors = [
    '#c0504d', # Red (strong negative correlations)
    '#d99694', # Light Red (medium negative correlations, less pinkish)
    '#e5b9b7', # Soft Red (weak negative correlations, less coral-like)
    '#f8e3e3', # White (neutral/zero correlations)
    '#b8cce4', # Light Sky Blue (weak positive correlations)
    '#95b3d7', # Medium Blue (medium positive correlations)
    '#366092', # Blue (strong positive correlations)
]


# Create the custom colormap
custom_cmap = LinearSegmentedColormap.from_list('graphic_inspired', graphic_colors)

# Normalize the colormap to the fixed range [-1, 1]
norm = Normalize(vmin=-1, vmax=1)

# Visualize the correlation matrix as a heatmap
plt.figure(figsize=(12, 8))
cax = plt.imshow(correlation_matrix, cmap=custom_cmap, norm=norm, interpolation='none')
plt.colorbar(cax, label='Correlation Coefficient')

# Set axis labels
plt.xticks(range(len(correlation_matrix.columns)), correlation_matrix.columns, rotation=0, ha='center', fontsize=10)
plt.yticks(range(len(correlation_matrix.columns)), correlation_matrix.columns, fontsize=10)

# Annotate the correlation values on the heatmap
for i in range(len(correlation_matrix.columns)):
    for j in range(len(correlation_matrix.columns)):
        correlation_value = correlation_matrix.iloc[i, j]

        # Use white color for correlations greater than 0.5, black for others
        text_color = 'white' if correlation_value > 0 else 'black'

        # Annotate with the correlation value in the appropriate color
        plt.text(j, i, f'{correlation_value:.2f}', ha='center', va='center', color=text_color, fontsize=12)

# Tight layout to ensure labels fit
plt.tight_layout()

# Add labels
plt.xlabel('Category Id.', fontsize=12)
plt.ylabel('Category Id.', fontsize=12)

# Save the heatmap as a PNG image
output_file = "CorrelationMatrixCategories.png"
plt.savefig(output_file, dpi=300, bbox_inches='tight')
plt.close()  # Close the plot to free memory

# Print the correlation matrix
print("Correlation Matrix:")
print(correlation_matrix)

# The correlation matrix is now saved as a PNG file
print(f"Heatmap saved as {output_file}")
