#!/bin/bash
aws s3 cp sample_goose.jpg s3://geeseornot/sample_goose.jpg
aws rekognition detect-labels --image '{"S3Object":{"Bucket":"geeseornot","Name":"sample_goose.jpg"}}' > objectsDetected.txt
aws s3 rm s3://geeseornot/sample_goose.jpg
cat objectsDetected.txt
#look for Goose in text file and get the confidence level: cat objectsDetected.txt | grep Goose -B 1
