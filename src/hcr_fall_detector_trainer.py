#!/usr/bin/env python
import rospy, sys, csv, os
from sklearn import svm

rospy.init_node('hcr_fall_detector_trainer', anonymous=True)

if __name__ == '__main__':
  # Build up X and Y
  X = []
  Y = []

  # Loop through files: fall_data_0.csv, fall_data_1.csv, etc.
  file_count = 0
  while (os.path.isfile("w/fall_data_"+str(file_count)+".csv")):
    file_name = "w/fall_data_"+str(file_count)+".csv"
    print("reading "+file_name)
    with open(file_name) as csvfile:
      reader = csv.reader(csvfile)
      # # x contains the current 10 samples
      # x = []
      for row in reader:
        row = [float(i) for i in row]
        y = row[-1]
        row = row[:-1]  # Remove y from row
        # x.append(row)
        # if (len(x) > 10):
        #   del x[0]
        # if (len(x) == 10):
        X.append(row)
        Y.append(y)
    file_count += 1
  if file_count == 0:
    print("error: no input CSV files found in w/ directory")
    sys.exit(0)

  clf=None
  if (len(X) > 0):
    clf = svm.SVC().fit(X,Y)

  if clf:
    x_test=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
    y_test=clf.predict(x_test)
    print(y_test)