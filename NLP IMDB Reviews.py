import pandas as pd
import numpy as np
import os
import io
import csv
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.feature_extraction.text import TfidfTransformer
from sklearn import linear_model
import re
from copy import deepcopy

test_path = "../resource/asnlib/public/imdb_te.csv"
train_path = "../resource/asnlib/public/aclImdb/train/"

if __name__ == "__main__":

 with open("stopwords.en.txt", "r") as f:
   content = f.read().splitlines()
  
 i=0
 labels={'pos':1,'neg':0}
 df=pd.DataFrame()
 for l in ('pos','neg'):
     path=os.path.join(train_path,l)
     for file in os.listdir(path):
       with io.open(os.path.join(path,file),'r',encoding='utf-8') as infile:
         txt=infile.read()
       df = df.append([[i,txt,labels[l]]],ignore_index=True)
       i=i+1
 df.columns=['row_number','text','polarity']
 df2=pd.DataFrame()
 df2=deepcopy(df)
 df2.to_csv('./imdb_tr.csv',index=False,encoding='utf-8')
 
 a=df["text"].size
 df1=pd.DataFrame()
 for i in range(0,a):
   txt=deepcopy(df["text"][i])
   txt1=re.sub("[^a-zA-Z]", " ", txt)
   txt2=txt1.lower().split()
   txt3=[j for j in txt2 if not j in content]
   txt4=" ".join(txt3)
   df1=df1.append([[i,txt4,df["polarity"][i]]],ignore_index=True)
 df1.columns=['row_number','text','polarity']

 voc = []
 for i in xrange(0,a):
    voc.append(df1["text"][i])

 vectorizer1 = CountVectorizer(max_features = 5000)   
 SGD1 = linear_model.SGDClassifier(loss='hinge',penalty='l1')
 unigram = vectorizer1.fit_transform(voc)
 unigram_model=SGD1.fit(unigram.toarray(), df1['polarity'])

 vectorizer2 = CountVectorizer(ngram_range=(1,2),max_features = 5000)  
 SGD2 = linear_model.SGDClassifier(loss='hinge',penalty='l1')
 bigram = vectorizer2.fit_transform(voc)
 bigram_model=SGD2.fit(bigram.toarray(),df1['polarity'])

 tfidf1 = TfidfTransformer(smooth_idf=False)
 SGD3 = linear_model.SGDClassifier(loss='hinge',penalty='l1')
 unigram_tfidf = tfidf1.fit_transform(unigram.toarray())
 unigramtfidf_model=SGD3.fit(unigram_tfidf.toarray(),df1['polarity'])

 tfidf2 = TfidfTransformer(smooth_idf=False)
 SGD4 = linear_model.SGDClassifier(loss='hinge',penalty='l1')
 bigram_tfidf = tfidf2.fit_transform(bigram.toarray())
 bigramtfidf_model=SGD4.fit(bigram_tfidf.toarray(),df1['polarity'])


 dftest=pd.DataFrame.from_csv(test_path, header=0, sep=',')

 c=dftest["text"].size
 dftest1=pd.DataFrame()
 for i in range(0,c):
   txt=deepcopy(dftest["text"][i])
   txt1=re.sub("[^a-zA-Z]", " ", txt)
   txt2=txt1.lower().split()
   txt3=[j for j in txt2 if not j in content]
   txt4=" ".join(txt3)
   dftest1=dftest1.append([[i,txt4]],ignore_index=True)
 dftest1.columns=['row_number','text']

 voctest = []
 for i in xrange(0,c):
     voctest.append(dftest1["text"][i])

 unigram_test=vectorizer1.transform(voctest)
 unigram_prediction=unigram_model.predict(unigram_test.toarray())
 unigram_output = pd.DataFrame( data={"polarity":unigram_prediction} )
 np.savetxt('unigram.output.txt', unigram_output, fmt='%d')

 bigram_test=vectorizer2.transform(voctest)
 bigram_prediction=bigram_model.predict(bigram_test.toarray())
 bigram_output=pd.DataFrame(data={"polarity":bigram_prediction})
 np.savetxt('bigram.output.txt', bigram_output, fmt='%d')

 unigram_tfidf_test=tfidf1.transform(unigram_test.toarray())
 unigram_tfidf_prediction=unigramtfidf_model.predict(unigram_tfidf_test.toarray())
 unigram_tfidf_output=pd.DataFrame(data={"polarity":unigram_tfidf_prediction})
 np.savetxt('unigramtfidf.output.txt', unigram_tfidf_output,fmt='%d')

 bigram_tfidf_test=tfidf2.transform(bigram_test.toarray())
 bigram_tfidf_prediction=bigramtfidf_model.predict(bigram_tfidf_test.toarray())
 bigram_tfidf_output=pd.DataFrame(data={"polarity":bigram_tfidf_prediction})
 np.savetxt('bigramtfidf.output.txt', bigram_tfidf_output,fmt='%d')
 
 pass
