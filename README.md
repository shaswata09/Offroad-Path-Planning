# Offroad-Path-Planning

## Datasets

All used datasets can be found at the following links with associated access procedures. 

1. [Massachusetts Road Dataset](https://www.kaggle.com/datasets/balraj98/massachusetts-roads-dataset)
2. [DeepGlobe Dataset](https://www.kaggle.com/datasets/balraj98/deepglobe-road-extraction-dataset)
3. [CAVS Dataset](https://www.kaggle.com/datasets/mitrashaswata/msstate-cavs-off-road-aerial-images)


## How to run Image Segmentation models

All image segmentation training and testing codes are written in python notebook files and can be found under [Colab](https://github.com/shaswata09/Offroad-Path-Planning/tree/main/Colab) directory and can be run through Google Colab (Online), Jupyter Notebook, or any suitable IDE.

## Aerial Traversability Prediction Data

Pre-computed segmentation results can be downloaded from the following Google Drive links. 

1. [Massachusetts Road Dataset](https://drive.google.com/drive/folders/1c9Wkf4DRLSn3JyNIC06im05jlshSLAoo)
2. [DeepGlobe Dataset](https://drive.google.com/drive/folders/1_rOJaJplyK_rslAu2TQAsZMtyGNWBiR7)
3. [CAVS Dataset](https://drive.google.com/drive/folders/179PggCbNTn1d1Uibq2OvZ_zkb2lkzg2w)

Each dataset comes with 4 subfolders:
* the input images
* binary traversability predictions
* binary traversability ground truths
* traversability probability matrices

## Uncertainty-aware Replanning A\* (URA\*)

To run URA\* in static mode (i.e. to generate the initial path) for the Massachusetts Road Dataset:
    
    python main.py --static_mode --start_goal data/massachusetts_start_goal.csv --segmented_dir data/sample_predictions_ensemble_massachuests/predictions --ground_truth_dir data/sample_predictions_ensemble_massachuests/ground_truth/ --prediction_dir data/sample_predictions_ensemble_massachuests/prediction_matrix/ --output_dir output --output_csv output/massachusetts.csv

This command will save the path planning results to the output directory and save evaluation metrics in an output CSV file.

## Calculating Metrics

To calculate aggregate evaluation metrics for each algorithm:

    python calculate_metrics.py --static_mode --csv massachusetts.csv
