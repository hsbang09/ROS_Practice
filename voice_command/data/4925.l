Language model created by QuickLM on Wed Oct 18 20:02:17 EDT 2017
Copyright (c) 1996-2010 Carnegie Mellon University and Alexander I. Rudnicky

The model is in standard ARPA format, designed by Doug Paul while he was at MITRE.

The code that was used to produce this language model is available in Open Source.
Please visit http://www.speech.cs.cmu.edu/tools/ for more information

The (fixed) discount mass is 0.5. The backoffs are computed using the ratio method.
This model based on a corpus of 5 sentences and 7 words

\data\
ngram 1=7
ngram 2=10
ngram 3=9

\1-grams:
-0.8808 </s> -0.3010
-0.8808 <s> -0.2398
-1.2788 BOTTOM -0.2527
-1.5798 CENTER -0.2398
-1.2788 LEFT -0.2398
-1.2788 RIGHT -0.2398
-1.2788 TOP -0.2527

\2-grams:
-0.6990 <s> BOTTOM 0.0000
-1.0000 <s> CENTER 0.0000
-0.6990 <s> TOP 0.0000
-0.6021 BOTTOM LEFT 0.0000
-0.6021 BOTTOM RIGHT 0.0000
-0.3010 CENTER </s> -0.3010
-0.3010 LEFT </s> -0.3010
-0.3010 RIGHT </s> -0.3010
-0.6021 TOP LEFT 0.0000
-0.6021 TOP RIGHT 0.0000

\3-grams:
-0.6021 <s> BOTTOM LEFT
-0.6021 <s> BOTTOM RIGHT
-0.3010 <s> CENTER </s>
-0.6021 <s> TOP LEFT
-0.6021 <s> TOP RIGHT
-0.3010 BOTTOM LEFT </s>
-0.3010 BOTTOM RIGHT </s>
-0.3010 TOP LEFT </s>
-0.3010 TOP RIGHT </s>

\end\
