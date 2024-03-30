import numpy as np 

def calc_matrix(x,y):
    # Compute means 
    x_mean = np.mean(x)
    y_mean = np.mean(y)

    # Compute std_dev
    std_dev_x = x - x_mean 
    std_dev_y = y - y_mean 

    # Compute sums of squares and cross-products
    sum_x = np.sum(std_dev_x ** 2)
    sum_y = np.sum(std_dev_y ** 2)
    sum_xy = np.sum(std_dev_x * std_dev_y)

    n = 10 

    cov_xx = sum_x / 10 
    cov_xy = sum_xy / 10 
    cov_yy = sum_y / 10 

    # Covariance matrix 
    cov_matrix = np.array([[cov_xx, cov_xy],
                          [cov_xy, cov_yy]])
    
    return cov_matrix 

# Data
data = [
    (-0.185731, 0.133793),
    (-0.93289, 0.103625),
    (-0.158778, 0.118960),
    (-0.146612, 0.186450),
    (-0.183889, 0.129306),
    (-0.175502, 0.166422),
    (-0.027864, 0.264203),
    (-0.169640, 0.444362),
    (-0.102736, 0.157829),
    (-0.112396, 0.132356),
    (-0.719062, -0.429203),
    (-0.456854, 0.395494),
    (0.555382, -0.118992),
    (-0.903472, -0.239280),
    (0.291698, 0.023429),
    (-0.070976, 0.069382),
    (-0.031259, 0.318662),
    (0.611579, -0.844062),
    (-0.346688, 0.557616),
    (-0.144909, 0.521361),    
    (0.171130, -0.371854),
    (-1.142071, 0.079925),
    (1.131518, 2.450634),
    (1.734908, -2.080539),
    (-1.116065, 1.264976),
    (0.6649949, -2.341614),
    (-0.473479, 3.321121),
    (1.857822, -0.311616),
    (3.826659, -0.235175),
    (0.490978, -0.843965)
]

# Splitting data into three separate arrays
data1 = np.array(data[:10])
data2 = np.array(data[10:20])
data3 = np.array(data[20:])

cov_matrix = calc_matrix(data1[:,0],data1[:,1])
print("Covariance Matrix:")
print(cov_matrix)
