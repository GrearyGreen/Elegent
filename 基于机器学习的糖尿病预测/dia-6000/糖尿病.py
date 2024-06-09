#导入常见且本项目用到的库
#pandas 是 Python 的核心数据分析支持库,提供了快速、灵活、明确的数据结构,旨在简单、直观地处理关系型、标记型数据。
#Numpy是一个扩展的程序库,支持维度数组和矩阵运算,还针对数组运算提供大量的数学函数库。
#matplotlib是提供数据绘图功能的第三方库,其pyplot子库主要用于实现各种数据展示图形的绘制。
#seaborn是基于Matplotlib的Python数据可视化库。它提供了一个高级界面,用于绘制引人入胜且内容丰富的统计图形。
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from lightgbm import LGBMClassifier


# 读取训练集数据，并输出其大小和训练集前几行
train = pd.read_csv('dia-6000/train.csv', encoding='gbk')
test = pd.read_csv('dia-6000/test.csv', encoding='gbk')
print('训练集的数据大小：',train.shape)
print('测试集的数据大小：',test.shape)#测试集无“血糖”这一列特征
print(train.head())
#查看字段类型
print(train.dtypes)
print(test.dtypes)


# 删除与数据分析无关的id列的所有行
train = train.drop(['id'], axis=1)  # axis=1表示沿着横轴方向删除

# 将性别映射为数字：男-1，女-0
train['性别'] = train['性别'].map({'男':1, '女':0})

# 根据血糖正常值标准，将血糖大于6.1定义为1，血糖小于3.9定义为0
data1 = train[train['血糖'] > 6.1]
data1['血糖'] = 1
data2 = train[train['血糖'] < 3.9]
data2['血糖'] = 0
df = pd.concat([data1, data2])  # 合并数据集

# 绘制柱状图和箱线图分析性别与血糖、年龄与血糖的关系
fig = plt.figure(figsize=(30, 14))  # 设置画布大小
plt.rcParams['font.sans-serif'] = 'SimHei'  # 设置中文显示
plt.rcParams['axes.unicode_minus'] = False

# 绘制柱状图，分析性别与血糖的关系
ax1 = fig.add_subplot(1, 2, 1)
sns.countplot(x='血糖', data=df, hue='性别', palette='rainbow')  # 使用seaborn库绘制柱状图，hue参数表示按性别进行颜色分组

# 绘制箱线图，分析年龄与血糖的关系
ax2 = fig.add_subplot(1, 2, 2)
sns.boxplot(x='血糖', y='年龄', data=df)  # 使用seaborn库绘制箱线图
plt.savefig('dia-6000/process/年龄与血糖的关系.jpg')
# plt.show()  # 显示图表



#相关性分析
numeric_columns = train.select_dtypes(include=['int','float']).columns
correlations = train[numeric_columns].corr()['血糖'].drop('血糖')#各特征值与血糖值的相关性分析
plt.figure(figsize=(50, 50))
sns.heatmap(train[numeric_columns].corr())#绘制相关热力图，查看总体特征相关性，可以通过热力图更清楚地看到相关性的强弱程度
plt.savefig('dia-6000/process/相关性分析.jpg')
# plt.show()
print(correlations)

# 读取数据
train = pd.read_csv('dia-6000/train.csv', encoding='gbk')
test = pd.read_csv('dia-6000/test.csv', encoding='gbk')


def make_feat(train, test):
    train_id = train.id.values.copy()  # 复制训练集的id列
    test_id = test.id.values.copy()  # 复制测试集的id列

    data = pd.concat([train, test])  # 合并训练集和测试集数据
    data['性别'] = data['性别'].map({'男': 1, '女': 0})  # 将性别映射为数字（男：1，女：0）

    data = data[data['年龄'] > 10]  # 筛选出年龄大于10岁的数据

    train_feat = data[data.id.isin(train_id)]  # 根据训练集的id筛选出相应的数据作为特征
    test_feat = data[data.id.isin(test_id)]  # 根据测试集的id筛选出相应的数据作为特征

    # 查看特征中的缺失值个数
    print(test_feat.isnull().sum(axis=0))#测试集
    print(train_feat.isnull().sum(axis=0))#训练集

    # 查看特征中的重复行
    print(test_feat.value_counts())

    # 删除缺失值很多的特征和无关分析的ID列
    train_feat = train_feat.drop(['id', '体检日期', '乙肝表面抗原', '乙肝表面抗体',
                                  '乙肝e抗原', '乙肝e抗体', '乙肝核心抗体'], axis=1)
    test_feat = test_feat.drop(['id','体检日期', '乙肝表面抗原', '乙肝表面抗体',
                                '乙肝e抗原','乙肝e抗体','乙肝核心抗体'], axis=1)
    # 对缺少一部分的数据进行中位数填充
    train_feat.fillna(train_feat.median(axis=0), inplace=True)
    test_feat.fillna(test_feat.median(axis=0), inplace=True)
    return train_feat, test_feat#这里不太懂
train_feat, test_feat = make_feat(train, test)
train_feat.to_csv('dia-6000/process/train_feat.csv', index=False, encoding='gbk')
test_feat.to_csv('dia-6000/process/test_feat.csv', index=False, encoding='gbk')

# 代码9-4
from sklearn.model_selection import KFold  # K折交叉验证函数
import lightgbm as lgb  # LightGBM是一个梯度Boosting框架，使用基于决策树的学习算法

# 读取处理后的数据
train_feat = pd.read_csv('dia-6000/train.csv', encoding = 'gbk')
test_feat = pd.read_csv('dia-6000/test.csv', encoding = 'gbk')
numeric_columns = train_feat.select_dtypes(include=['int','float']).columns
predictors = [f for f in train_feat[numeric_columns].columns if f not in ['血糖']]  # 提取列名
print('开始训练...')

# 训练模型并预测
def lgb_train(train_feat, test_feat):
    print('开始CV 5折训练...')
    train_preds = np.zeros(train_feat.shape[0])
    test_preds = np.zeros((test_feat.shape[0], 5))
    kf = KFold(n_splits=5, shuffle=True, random_state=4013)  # K折交叉验证
    for i, (train_index, test_index) in enumerate(kf.split(train_feat)):
        print('第{}次训练'.format(i))
        train_feat1 = train_feat.iloc[train_index]#选择训练集数值对应的索引
        train_feat2 = train_feat.iloc[test_index]

        # 创建LGBMRegressor模型
        model_lgb = lgb.LGBMRegressor(objective='regression', num_leaves=8,
                                      learning_rate=0.03, n_estimators=400,
                                      max_bin=30, bagging_fraction=0.8,
                                      bagging_freq=10, feature_fraction=0.5,
                                      feature_fraction_seed=10, bagging_seed=10,
                                      min_data_in_leaf=80, nthread=8,
                                      min_sum_hessian_in_leaf=0.2)#括号里是模型参数，可默认

        # 在训练集上进行拟合
        gbm = model_lgb.fit(train_feat1[predictors].values, train_feat1['血糖'].values)

        # 预测验证集的血糖值
        train_preds[test_index] += gbm.predict(train_feat2[predictors])

        # 预测测试集的血糖值
        test_preds[:, i] = gbm.predict(test_feat[predictors])

    return train_preds, test_preds


# 调用函数进行训练和预测
train_preds, test_preds = lgb_train(train_feat, test_feat)

# 保存数据
train_true_pred = pd.DataFrame({'true':train_feat['血糖'], 'pred':train_preds})
train_true_pred.to_csv('dia-6000/process/train_true_pred.csv', index=False, encoding = 'gbk')
test_preds1 = pd.DataFrame(test_preds.mean(axis=1))
test_preds1.columns = ['血糖']
test_preds1.to_csv('dia-6000/process/test_preds.csv', index=False, encoding = 'gbk')

# 代码9-5

train_true_pred = pd.read_csv('dia-6000/process/train_true_pred.csv', encoding='gbk')# 读取数据，encoding='gbk'可椟中文字符
# 选择前100个样本
train_true_pred = train_true_pred.iloc[:100, :]
x = range(len(train_true_pred))
y_true = train_true_pred['true']  # 提取血糖真实值
y_pred = train_true_pred['pred']  # 提取血糖预测值

# 绘制部分训练数据的血糖的真实值与预测值的折线图
plt.figure(figsize=(10, 6))
plt.plot(x, y_true)
plt.plot(x, y_pred)
# 设置rc参数显示中文标题
plt.rcParams['font.sans-serif'] = 'SimHei'  # 设置字体为SimHei显示中文
plt.ylabel('血糖')
plt.title('部分训练数据真实值与预测值结果的折线图')
plt.legend(("真实值","预测值"), loc=1)
plt.savefig('dia-6000/process/部分训练数据真实值与预测值结果的折线图.jpg')
# plt.show()
print()


# 对预测数据筛选出血糖值在3.9～6.1毫摩尔/升的数据，提醒他们注意血糖
test = pd.read_csv('dia-6000/test.csv', encoding='gbk')
test_preds = pd.read_csv('dia-6000/process/test_preds.csv', encoding='gbk')
data = pd.concat([test, test_preds], axis=1)
df1 = data[data['血糖'] > 6.7]
df1 = df1[np.isnan(df1['id']) == False]
df1.to_csv('dia-6000/process/hyperglycemia.csv', index=False, encoding='gbk')


# 代码9-7
from sklearn.metrics import mean_squared_error,r2_score,precision_score,classification_report,accuracy_score#导入评价指标库
# 计算均方误差回归损失
MSE1 = mean_squared_error(train_true_pred['true'], train_true_pred['pred'])
print('均方误差：  {}'.format(MSE1))





