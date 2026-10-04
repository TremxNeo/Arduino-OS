#include "../../../core_api.h"
#include <math.h>
#include <string.h>

static bool s_active=false;
static char s_expr[64]="";
static void graph_disconnect(void){s_active=false;}

struct Parser{const char*p;float x;bool error;};
static void ws(Parser&q){while(*q.p==' '||*q.p=='\t')++q.p;}
static bool id0(char c){return(c>='a'&&c<='z')||(c>='A'&&c<='Z')||c=='_';}
static bool idc(char c){return id0(c)||(c>='0'&&c<='9');}
static float expr(Parser&q); static float unary(Parser&q);
static float number(Parser&q){
  ws(q); const char*s=q.p; bool digit=false; float v=0,frac=0.1f;
  while(*q.p>='0'&&*q.p<='9'){digit=true;v=v*10+(*q.p++-'0');}
  if(*q.p=='.'){++q.p;while(*q.p>='0'&&*q.p<='9'){digit=true;v+=(*q.p++-'0')*frac;frac*=.1f;}}
  if(!digit){q.error=true;return 0;}
  if(*q.p=='e'||*q.p=='E'){
    const char*e=q.p++; bool neg=false;if(*q.p=='+'||*q.p=='-'){neg=*q.p=='-';++q.p;}
    int n=0;const char*d=q.p;while(*q.p>='0'&&*q.p<='9'){if(n<38)n=n*10+(*q.p-'0');++q.p;}
    if(q.p==d)q.p=e;else if(neg)n=-n; v*=pow(10.0f,(float)n);
  }
  return v;
}
static float primary(Parser&q){
  ws(q);
  if(*q.p=='('){++q.p;float v=expr(q);ws(q);if(*q.p==')')++q.p;else q.error=true;return v;}
  if((*q.p>='0'&&*q.p<='9')||*q.p=='.')return number(q);
  if(id0(*q.p)){
    char n[10];int k=0;while(idc(*q.p)){if(k<9)n[k++]=*q.p;++q.p;}n[k]=0;
    if(!strcmp(n,"x"))return q.x;
    if(!strcmp(n,"pi"))return 3.14159265f;
    if(!strcmp(n,"e"))return 2.71828183f;
    bool fn=!strcmp(n,"sin")||!strcmp(n,"cos")||!strcmp(n,"tan")||!strcmp(n,"sqrt")||!strcmp(n,"abs")||!strcmp(n,"exp")||!strcmp(n,"ln")||!strcmp(n,"log");
    if(!fn){q.error=true;return 0;}
    ws(q);bool par=*q.p=='(';if(par)++q.p;float a=par?expr(q):unary(q);ws(q);
    if(par){if(*q.p==')')++q.p;else q.error=true;}
    if(!strcmp(n,"sin"))return sin(a);if(!strcmp(n,"cos"))return cos(a);
    if(!strcmp(n,"tan"))return tan(a);if(!strcmp(n,"sqrt"))return sqrt(a);
    if(!strcmp(n,"abs"))return fabs(a);if(!strcmp(n,"exp"))return exp(a);
    if(!strcmp(n,"ln"))return log(a);if(!strcmp(n,"log"))return log10(a);
    return 0;
  }
  q.error=true;return 0;
}
static float power(Parser&q){float a=primary(q);ws(q);if(*q.p=='^'){++q.p;a=pow(a,unary(q));}return a;}
static float unary(Parser&q){ws(q);if(*q.p=='+'){++q.p;return unary(q);}if(*q.p=='-'){++q.p;return -unary(q);}return power(q);}
static bool starts(Parser&q){ws(q);return id0(*q.p)||*q.p=='('||*q.p=='.'||(*q.p>='0'&&*q.p<='9');}
static float term(Parser&q){float v=unary(q);while(!q.error){ws(q);if(*q.p=='*'){++q.p;v*=unary(q);}else if(*q.p=='/'){++q.p;v/=unary(q);}else if(starts(q))v*=unary(q);else break;}return v;}
static float expr(Parser&q){float v=term(q);while(!q.error){ws(q);if(*q.p=='+'){++q.p;v+=term(q);}else if(*q.p=='-'){++q.p;v-=term(q);}else break;}return v;}
static float eval(float x){Parser q={s_expr,x,false};float v=expr(q);ws(q);if(*q.p)q.error=true;return q.error?NAN:v;}

static int pix(float y,float lo,float hi){if(!isfinite(y)||y<lo||y>hi)return-1;return(int)((hi-y)*41.0f/(hi-lo)+.5f);}
static void graph_draw(void){
  const int W=54,H=21,P=42; const float xmin=-10,xmax=10;
  float lo=1e30f,hi=-1e30f;int n=0;
  for(int i=0;i<=240;i++){float x=xmin+(xmax-xmin)*i/240.0f,y=eval(x);if(isfinite(y)&&fabs(y)<1e6f){if(y<lo)lo=y;if(y>hi)hi=y;++n;}}
  if(!n){lo=-1;hi=1;} float span=hi-lo;if(span<.001f){float c=(hi+lo)*.5f;span=2;lo=c-1;hi=c+1;}else{float pad=span*.08f;lo-=pad;hi+=pad;}
  Serial.print(F("\033[2J\033[Hf(x)= "));Serial.println(s_expr);
  for(int r=0;r<H;r++){
    int a=r*2,b=a+1;
    for(int c=0;c<W;c++){
      float x=xmin+(xmax-xmin)*c/(W-1.0f),y=eval(x);int p=pix(y,lo,hi);
      bool curve=false;
      static int prev[54];
      int last=(c?prev[c-1]:-1);
      if(p>=0){curve=(p==a||p==b);if(last>=0){int u=last<p?last:p,d=last<p?p:last;curve|=(a>=u&&a<=d)||(b>=u&&b<=d);}}
      prev[c]=p;
      bool top=curve&&p==a,bot=curve&&p==b;
      if(last>=0&&p>=0){int u=last<p?last:p,d=last<p?p:last;if(a>=u&&a<=d)top=true;if(b>=u&&b<=d)bot=true;}
      bool xaxis=(lo<=0&&hi>=0&&fabs(a-(hi*41/(hi-lo)))<1.0f)||(lo<=0&&hi>=0&&fabs(b-(hi*41/(hi-lo)))<1.0f);
      bool yaxis=c==W/2;
      if(top&&bot)Serial.print(F("█"));else if(top)Serial.print(F("▀"));else if(bot)Serial.print(F("▄"));
      else if(xaxis&&yaxis)Serial.print(F("┼"));else if(yaxis)Serial.print(F("│"));else if(xaxis)Serial.print(F("─"));else Serial.print(' ');
    }Serial.println();
  }
  Serial.print(F("x=-10..10  y="));Serial.print(lo,1);Serial.print(F(".."));Serial.println(hi,1);
  core_prompt_refresh();
}
static void help(void){Serial.println(F("<expr> | help | clear | /"));Serial.println(F("+ - * / ^ ( )  sin cos tan sqrt abs exp ln log  pi e"));}
static bool graph_open(void){s_active=true;core_clear_screen();Serial.println(F("== GRAPH =="));Serial.println(F("Enter an expression | / = leave"));core_prompt_refresh();return true;}
static bool graph_line(char*line){
  if(!s_active)return false;if(!line[0]){core_prompt_refresh();return true;}if(!strcmp(line,"/")){s_active=false;core_prompt_refresh();return true;}
  if(!strcmp(line,"help")){help();core_prompt_refresh();return true;}if(!strcmp(line,"clear")){core_clear_screen();core_prompt_refresh();return true;}
  strncpy(s_expr,line,sizeof(s_expr)-1);s_expr[sizeof(s_expr)-1]=0;graph_draw();return true;
}
static const AppHooks graph_hooks={"graph","ANSI graphing calculator",graph_disconnect,graph_open,nullptr,graph_line,nullptr,0,nullptr};
static const AppRegistrar graph_auto_register(&graph_hooks);
