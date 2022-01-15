#ifndef helpers_h
#define helpers_h

class FilterExpRunningAverage{
private:
    float _fil_val;
public:
    FilterExpRunningAverage(){}
    ~FilterExpRunningAverage(){};
    
    float filter(float new_val){
      float k = abs(new_val - _fil_val) / 100.0;
      if (k > 1) k = 1.0;
      else if (k < 0.1) k = 0.1; 
      _fil_val += (new_val - _fil_val) * k;
      return _fil_val;  
    }
};

#endif
