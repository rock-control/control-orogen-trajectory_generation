echo "Running simulation..."
ruby test_interpolator.rb
echo "Running simulation... done"

echo "Extracting log data..."
ruby extract_csv.rb
echo "Extracting log data... done"

echo "Plotting..."
source plot_all.sh
echo "Plotting... done"

