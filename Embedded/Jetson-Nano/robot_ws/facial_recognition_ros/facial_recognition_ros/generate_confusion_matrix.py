import matplotlib.pyplot as plt
import numpy as np

# ======== STEP 1: GET USER INPUT ========
print("Enter test parameters to generate your labeled confusion matrix.\n")

target_name = input("Target identity (e.g., Yahir): ").strip()
environment = input("Environment label (e.g., indoor_bright): ").strip()

print("\nNow enter your confusion matrix counts:")
TP = int(input("True Positives (TP): "))
FP = int(input("False Positives (FP): "))
FN = int(input("False Negatives (FN): "))
TN = int(input("True Negatives (TN): "))

# ======== STEP 2: FORMAT MATRIX ========
cm = np.array([[TN, FP],
               [FN, TP]])

labels = [f"Not {target_name}", target_name]
total = cm.sum()

cell_titles = [["True Negative", "False Positive"],
               ["False Negative", "True Positive"]]

annot = np.empty_like(cm).astype(str)
for i in range(cm.shape[0]):
    for j in range(cm.shape[1]):
        pct = 100 * cm[i, j] / total
        annot[i, j] = f"{cell_titles[i][j]}\n{pct:.1f}%"

# ======== STEP 3: PLOT ========
fig, ax = plt.subplots(figsize=(6, 5))
ax.matshow(cm, cmap='Blues')

# Matrix cell labels
for (i, j), val in np.ndenumerate(annot):
    ax.text(j, i, val, ha='center', va='center', fontsize=11, weight='bold')

# Axis ticks and labels
ax.set_xticks([0, 1])
ax.set_yticks([0, 1])
ax.set_xticklabels(labels, fontweight='bold')
ax.set_yticklabels(labels, fontweight='bold')

# Axis titles and figure title
plt.xlabel("Predicted Label", fontweight='bold', fontsize=11)
plt.ylabel("True Label", fontweight='bold', fontsize=11)
plt.title(f"Confusion Matrix � {target_name} ({environment})", fontweight='bold', fontsize=13)

plt.tight_layout()
plt.show()

# ======== STEP 4: OPTIONAL SAVE ========
save = input("Save this figure as an image file? (y/n): ").strip().lower()
if save == 'y':
    filename = f"conf_matrix_{target_name}_{environment}.png".replace(" ", "_")
    fig.savefig(filename, dpi=300)
    print(f"Saved as {filename}")
