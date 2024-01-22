from openpilot.common.params_pyx import Params, ParamKeyType, UnknownKeyName
assert Params
assert ParamKeyType
assert UnknownKeyName

if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")

"""
SConscript 파일에서 # Cython bindings라고 주석 처리된 섹션을 볼 수 있습니다. 
이 스크립트의 부분은 params_pyx.pyx 소스 파일에서 
params_pyx.so 공유 객체 파일을 빌드하기 위한 Cython 컴파일 과정을 정의합니다. 
이 공유 객체 파일은 params 모듈을 위해 작성된 기본 C++ 코드에 Python 바인딩을 제공하는 데 사용됩니다.

이것은 SCons라는 소프트웨어 구축 도구를 사용하는 빌드 구성 파일입니다. 
params_pyx.pyx Cython 파일을 컴파일하고 
다른 C++ 라이브러리(_common, zmq, json11)와 연결하여 
Python에서 가져오고 사용할 수 있는 
params_pyx.so 라이브러리를 생성하는 방법을 정의합니다.

params.py 파일은 
컴파일된 Cython 모듈 params_pyx를 가져와서 
해당 내부에 정의된 Params 클래스와 상호작용하는 
명령줄 인터페이스를 제공하는 Python 스크립트입니다. 
이 스크립트를 적절한 인수와 함께 호출함으로써 
사용자는 명령줄에서 파라미터를 가져오고 설정할 수 있습니다.

따라서 params_pyx.cpp 파일은 
SConscript 파일에서 정의된 빌드 과정의 일부로 생성되었을 가능성이 높으며, 
Cython은 .pyx 파일들을 
Python 프로그램에서 가져올 수 있는 
.so (또는 윈도우에서는 .pyd) 공유 라이브러리로 컴파일하는 데 사용됩니다.
"""