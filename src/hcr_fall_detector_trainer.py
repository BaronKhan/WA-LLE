#!/usr/bin/env python
import rospy, sys, csv, os, pickle
from sklearn import svm

rospy.init_node('hcr_fall_detector_trainer', anonymous=True)

if __name__ == '__main__':
  # Build up X and Y
  X = []  # List of fall data
  Y = []  # List of corresponding falling states (0 or 1)

  # Loop through files: fall_data_0.csv, fall_data_1.csv, etc.
  file_count = 0
  while (os.path.isfile("w/fall_data_"+str(file_count)+".csv")):
    file_name = "w/fall_data_"+str(file_count)+".csv"
    print("reading "+file_name)
    with open(file_name) as csvfile:
      reader = csv.reader(csvfile)
      for row in reader:
        row = [float(i) for i in row]
        y = row[-1]
        row = row[:-1]  # Remove y from row
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
    # Test model
    # x_test=[[1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
    # y_test=clf.predict(x_test)
    # print(y_test)

    # Save model using pickle
    # Let's not overwrite existing models just in case
    file_count = 0
    while (os.path.isfile("w/svm_"+str(file_count)+".mdl")):
      file_count += 1
    file_name = "w/svm_"+str(file_count)+".mdl"
    with open(file_name, 'wb') as fp:
      pickle.dump(clf, fp, protocol=1)
    print("saved model to "+file_name)

    # Test deserialisation
    # clf = pickle.load( open( file_name, "rb" ) )
    # x_test=[[1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
    # y_test=clf.predict(x_test)
    # print(y_test)