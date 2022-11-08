# Artifact Reconciliation

## Analysing reconciliation performance

In the config file artifact_reconciliation.yaml set the analyse_results 
parameter to true in order to save a csv which will contain on each line:

- artiact name, artifact parent

The file path of this csv is set in the config file under results_file.


WARNING: 
  - Since we are apending a file, please make sure that the 'results_file'
does not exist prior to running the code
  - This has been developed for a single robot. 
