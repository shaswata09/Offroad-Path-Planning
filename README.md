# Offroad-Path-Planning

## Aerial Traversability Prediction Data

Pre-computed segmentation results can be downloaded from the following Google Drive links. Each dataset comes with 4 subfolders:
(i) the input images, (ii) binary traversability predictions (iii) binary traversability ground truths (iv) traversability probability matrices

1. [Massachusetts Road Dataset](https://drive.google.com/drive/folders/1c9Wkf4DRLSn3JyNIC06im05jlshSLAoo)
2. [DeepGlobe Dataset](https://drive.google.com/drive/folders/1_rOJaJplyK_rslAu2TQAsZMtyGNWBiR7)
3. [CAVS Dataset](https://drive.google.com/drive/folders/1neBkxoPbUi7Y8gX6EH0LHgNEELd4M7Lv)

## Uncertainty-aware Replanning A\* (URA\*)

To run URA\* in static mode (i.e. to generate the initial path) for the Massachusetts Road Dataset:
    
    python main.py --static_mode --start_goal data/massachusetts_start_goal.csv --segmented_dir data/sample_predictions_ensemble_massachuests/predictions --ground_truth_dir data/sample_predictions_ensemble_massachuests/ground_truth/ --prediction_dir data/sample_predictions_ensemble_massachuests/prediction_matrix/ --output_dir output --output_csv output/massachusetts.csv

This command will save the path planning results to the output directory and save evaluation metrics in an output CSV file.

## Calculating Metrics

To calculate aggregate evaluation metrics for each algorithm:

    python calculate_metrics.py --static_mode --csv massachusetts.csv
