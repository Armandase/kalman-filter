# x̄k = x̄(k - 1) + (xk - x(k-n))/n
def mobile_average(data, k=0, prev_avg=0, batch_size=10):
    values = []
    for k in range(len(data)):
        if k < batch_size:
            if k == 0:
                prev_avg = data[k]
            else:
                # if k is lower than batch size, calculate the average
                prev_avg = ((k - 1.) / k) * prev_avg + (1. / k) * data[k]
            values.append(prev_avg)
        else:
            prev_avg = prev_avg + (data[k] - data[k - batch_size]) / batch_size
            values.append(prev_avg)
    return values

# x̄k+1 = α * x̄k + 1-α * xk
def low_pass_filter(data, k=0, prev_avg=0, alpha=0.9):
    values = []
    for k in range(len(data)):
        if k == 0:
            prev_avg = data[k]
        prev_avg = alpha * prev_avg + (1 - alpha) * data[k]
        values.append(prev_avg)
    return values