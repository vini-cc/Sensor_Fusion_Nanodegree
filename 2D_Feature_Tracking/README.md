# 2D Feature Tracking

Name: Vinícius Costa
Florianopolis, June 09 2021.
Try #1

## Results


| MATCHER_TYPE | DESCRIPTOR_TYPE | SELECTOR_TYPE | DETECTOR | DESCRIPTOR | TIME_DET[ms] | TIME_DESC[ms] | TIME_TOTAL[ms] | KEYP_EXTRACTION | KEYP_MATCH | % MATCH |
|--------------|-----------------|---------------|----------|------------|--------------|---------------|----------------|-----------------|------------|---------|
| BF           | BINARY          | KNN           | AKAZE    | AKAZE      | 582.25       | 19.07         | 601,92         | 1670            | 1491       | 89,28%  |
| BF           | BINARY          | KNN           | AKAZE    | BRIEF      | 584.82       | 20.06         | 604.88         | 1670            | 1491       | 89,28%  |
| BF           | BINARY          | KNN           | AKAZE    | BRISK      | 583.87       | 19.45         | 603.32         | 1670            | 1491       | 89,28%  |
| BF           | BINARY          | KNN           | AKAZE    | FREAK      | 578.72       | 19.55         | 598.27         | 1670            | 1491       | 89,28%  |
| BF           | BINARY          | KNN           | AKAZE    | ORB        | 585.53       | 19.77         | 605.30         | 1670            | 1491       | 89,28%  |
| BF           | BINARY          | KNN           | AKAZE    | SIFT       | 590.51       | 20.07         | 610.58         | 1670            | 1491       | 89,28%  |
|              |                 |               |          |            |              |               |                |                 |            |         |
| BF           | BINARY          | KNN           | BRISK    | AKAZE      | 356.69       | 29.91         | 386.60         | 2762            | 2508       | 90.80%  |
| BF           | BINARY          | KNN           | BRISK    | BRIEF      | 363.80       | 29.78         | 393.58         | 2762            | 2508       | 90.80%  |
| BF           | BINARY          | KNN           | BRISK    | BRISK      | 368.49       | 30.40         | 398.90         | 2762            | 2508       | 90.80%  |
| BF           | BINARY          | KNN           | BRISK    | FREAK      | 362.01       | 29.73         | 391.74         | 2762            | 2508       | 90.80%  |
| BF           | BINARY          | KNN           | BRISK    | ORB        | 361.91       | 29.45         | 391.36         | 2762            | 2508       | 90.80%  |
| BF           | BINARY          | KNN           | BRISK    | SIFT       | 363.58       | 29.64         | 393.22         | 2762            | 2508       | 90.80%  |
|              |                 |               |          |            |              |               |                |                 |            |         |
| BF           | BINARY          | KNN           | FAST     | AKAZE      | 13.56        | 19.36         | 32.01          | 1491            | 1348       | 90.41%  |
| BF           | BINARY          | KNN           | FAST     | BRIEF      | 13.60        | 18.68         | 32.28          | 1491            | 1348       | 90.41%  |
| BF           | BINARY          | KNN           | FAST     | BRISK      | 13.31        | 19.29         | 32.59          | 1491            | 1348       | 90.41%  |
| BF           | BINARY          | KNN           | FAST     | FREAK      | 13.47        | 18.86         | 32.33          | 1491            | 1348       | 90.41%  |
| BF           | BINARY          | KNN           | FAST     | ORB        | 13.17        | 18.61         | 31.78          | 1491            | 1348       | 90.41%  |
| BF           | BINARY          | KNN           | FAST     | SIFT       | 13.55        | 19.03         | 32.59          | 1491            | 1348       | 90.41%  |
|              |                 |               |          |            |              |               |                |                 |            |         |
| BF           | BINARY          | KNN           | HARRIS   | AKAZE      | 191.05       | 9.72          | 200.78         | 248             | 214        | 86.29%  |
| BF           | BINARY          | KNN           | HARRIS   | BRIEF      | 208.82       | 9.75          | 218.58         | 248             | 214        | 86.29%  |
| BF           | BINARY          | KNN           | HARRIS   | BRISK      | 192.75       | 9.67          | 202.42         | 248             | 214        | 86.29%  |
| BF           | BINARY          | KNN           | HARRIS   | FREAK      | 198.15       | 9.46          | 207.61         | 248             | 214        | 86.29%  |
| BF           | BINARY          | KNN           | HARRIS   | ORB        | 204.62       | 9.68          | 214.30         | 248             | 214        | 86.29%  |
| BF           | BINARY          | KNN           | HARRIS   | SIFT       | 215.87       | 11.55         | 227.00         | 248             | 214        | 86.29%  |
|              |                 |               |          |            |              |               |                |                 |            |         |
| BF           | BINARY          | KNN           | ORB      | AKAZE      | 91.81        | 13.64         | 105.45         | 1161            | 950        | 81.83%  |
| BF           | BINARY          | KNN           | ORB      | BRIEF      | 91.54        | 13.70         | 105.24         | 1161            | 950        | 81.83%  |
| BF           | BINARY          | KNN           | ORB      | BRISK      | 94.09        | 13.67         | 107.76         | 1161            | 950        | 81.83%  |
| BF           | BINARY          | KNN           | ORB      | FREAK      | 95.75        | 13.64         | 109.39         | 1161            | 950        | 81.83%  |
| BF           | BINARY          | KNN           | ORB      | ORB        | 94.29        | 13.81         | 108.10         | 1161            | 950        | 81.83%  |
| BF           | BINARY          | KNN           | ORB      | SIFT       | 100.23       | 14.85         | 115.08         | 1161            | 950        | 81.83%  |
|              |                 |               |          |            |              |               |                |                 |            |         |
| BF           | BINARY          | KNN           | SHITOMASI| AKAZE      | 176.34       | 18.74         | 195.07         | 1179            | 1067       | 90.50%  |
| BF           | BINARY          | KNN           | SHITOMASI| BRIEF      | 201.12       | 18.53         | 219.65         | 1179            | 1067       | 90.50%  |
| BF           | BINARY          | KNN           | SHITOMASI| BRISK      | 186.85       | 18.62         | 205.46         | 1179            | 1067       | 90.50%  |
| BF           | BINARY          | KNN           | SHITOMASI| FREAK      | 175.56       | 18.17         | 193.73         | 1179            | 1067       | 90.50%  |
| BF           | BINARY          | KNN           | SHITOMASI| ORB        | 186.82       | 19.11         | 205.93         | 1179            | 1067       | 90.50%  |
| BF           | BINARY          | KNN           | SHITOMASI| SIFT       | 185.24       | 18.07         | 203.28         | 1179            | 1067       | 90.50%  |
|              |                 |               |          |            |              |               |                |                 |            |         |
| BF           | BINARY          | KNN           | SIFT     | AKAZE      | 984.40       | 18.53         | 1002.93        | 1386            | 1248       | 90.04%  |
| BF           | BINARY          | KNN           | SIFT     | BRIEF      | 982.45       | 17.90         | 1000.35        | 1386            | 1248       | 90.04%  |
| BF           | BINARY          | KNN           | SIFT     | BRISK      | 982.74       | 17.98         | 1000.72        | 1386            | 1248       | 90.04%  |
| BF           | BINARY          | KNN           | SIFT     | FREAK      | 988.52       | 18.25         | 1006.77        | 1386            | 1248       | 90.04%  |
| BF           | BINARY          | KNN           | SIFT     | ORB        | 990.40       | 17.78         | 1008.17        | 1386            | 1248       | 90.04%  |
| BF           | BINARY          | KNN           | SIFT     | SIFT       | 986.90       | 18.42         | 1005.32        | 1386            | 1248       | 90.04%  |
