try {
    // Pre-initialize network
    cout << "Warming up the network..." << endl;
    Mat dummy(300, 300, CV_8UC3, Scalar(0, 0, 0));
    Mat inputBlob;
    blobFromImage(dummy, inputBlob, 1/127.5, Size(300, 300), 
             Scalar(127.5, 127.5, 127.5), true, false);
    mNet.setInput(inputBlob);
    
    vector<Mat> dummy;
    // Fix: Replace vector<int> with explicit dimensions
    Mat dummyInput = Mat::zeros(1, 3, CV_32F); // Corrected call to zeros
    mNet.setInput(dummyInput);
    mNet.forward(dummy, getOutputsNames(mNet));
    cout << "Network warmup successful" << endl;
} catch (const cv::Exception& e) {
    cerr << "Exception during warmup forward pass: " << e.what() << endl;
    cerr << "This may indicate a model compatibility issue" << endl;
}