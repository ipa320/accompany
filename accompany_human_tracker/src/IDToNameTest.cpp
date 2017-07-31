

#include <IDToName.h>
#include <iostream>
using namespace std;

int main()
{
  IDToName itn;

  cout<<itn<<endl;

  while (true)
  {
    string name;
    int id;
    cout<<"enter name:"<<endl;
    cin>>name;
    cout<<"enter id:"<<endl;
    cin>>id;
    
    itn.setIDName(id,name);
    cout<<endl<<itn<<endl;
  }

}

