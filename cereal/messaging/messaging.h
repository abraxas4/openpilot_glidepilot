#pragma once

// MJ

#include <cstddef>
#include <map>
#include <string>
#include <vector>
#include <utility>
#include <time.h>

#include <capnp/serialize.h>

#include "cereal/gen/cpp/log.capnp.h"

#ifdef __APPLE__
#define CLOCK_BOOTTIME CLOCK_MONOTONIC
#endif

#define MSG_MULTIPLE_PUBLISHERS 100

bool messaging_use_zmq();

class Context {
public:
  virtual void * getRawContext() = 0;
  static Context * create();
  virtual ~Context(){}
};

class Message {
public:
  virtual void init(size_t size) = 0;
  virtual void init(char * data, size_t size) = 0;
  virtual void close() = 0;
  virtual size_t getSize() = 0;
  virtual char * getData() = 0;
  virtual ~Message(){}
};


class SubSocket {
public:
  virtual int connect(Context *context, std::string endpoint, std::string address, bool conflate=false, bool check_endpoint=true) = 0;
  virtual void setTimeout(int timeout) = 0;
  virtual Message *receive(bool non_blocking=false) = 0;
  virtual void * getRawSocket() = 0;
  static SubSocket * create();
  static SubSocket * create(Context * context, std::string endpoint, std::string address="127.0.0.1", bool conflate=false, bool check_endpoint=true);
  virtual ~SubSocket(){}
};

class PubSocket {
public:
  virtual int connect(Context *context, std::string endpoint, bool check_endpoint=true) = 0;
  virtual int sendMessage(Message *message) = 0;
  virtual int send(char *data, size_t size) = 0;
  virtual bool all_readers_updated() = 0;
  static PubSocket * create();
  static PubSocket * create(Context * context, std::string endpoint, bool check_endpoint=true);
  static PubSocket * create(Context * context, std::string endpoint, int port, bool check_endpoint=true);
  virtual ~PubSocket(){}
};

class Poller {
public:
  virtual void registerSocket(SubSocket *socket) = 0;
  virtual std::vector<SubSocket*> poll(int timeout) = 0;
  static Poller * create();
  static Poller * create(std::vector<SubSocket*> sockets);
  virtual ~Poller(){}
};

class SubMaster {
public:
  SubMaster(const std::vector<const char *> &service_list, const std::vector<const char *> &poll = {},
            const char *address = nullptr, const std::vector<const char *> &ignore_alive = {});
  void update(int timeout = 1000);
  void update_msgs(uint64_t current_time, const std::vector<std::pair<std::string, cereal::Event::Reader>> &messages);
  inline bool allAlive(const std::vector<const char *> &service_list = {}) { return all_(service_list, false, true); }
  inline bool allValid(const std::vector<const char *> &service_list = {}) { return all_(service_list, true, false); }
  inline bool allAliveAndValid(const std::vector<const char *> &service_list = {}) { return all_(service_list, true, true); }
  void drain();
  ~SubMaster();

  uint64_t frame = 0;
  bool updated(const char *name) const;
  bool alive(const char *name) const;
  bool valid(const char *name) const;
  uint64_t rcv_frame(const char *name) const;
  uint64_t rcv_time(const char *name) const;
  cereal::Event::Reader &operator[](const char *name) const;

private:
  bool all_(const std::vector<const char *> &service_list, bool valid, bool alive);
  Poller *poller_ = nullptr;
  struct SubMessage;
  std::map<SubSocket *, SubMessage *> messages_;
  std::map<std::string, SubMessage *> services_;
};

class MessageBuilder : public capnp::MallocMessageBuilder {
public:
  MessageBuilder() = default;

  cereal::Event::Builder initEvent(bool valid = true) {
    cereal::Event::Builder event = initRoot<cereal::Event>();
    struct timespec t;
    clock_gettime(CLOCK_BOOTTIME, &t);
    uint64_t current_time = t.tv_sec * 1000000000ULL + t.tv_nsec;
    event.setLogMonoTime(current_time);
    event.setValid(valid);
    return event;
  }

  kj::ArrayPtr<capnp::byte> toBytes() {
    heapArray_ = capnp::messageToFlatArray(*this);
    return heapArray_.asBytes();
  }

  size_t getSerializedSize() {
    return capnp::computeSerializedSizeInWords(*this) * sizeof(capnp::word);
  }

  int serializeToBuffer(unsigned char *buffer, size_t buffer_size) {
    size_t serialized_size = getSerializedSize();
    if (serialized_size > buffer_size) { return -1; }
    kj::ArrayOutputStream out(kj::ArrayPtr<capnp::byte>(buffer, buffer_size));
    capnp::writeMessage(out, *this);
    return serialized_size;
  }

private:
  kj::Array<capnp::word> heapArray_;
};

// MJ
class PubMaster {
public:
  PubMaster(const std::vector<const char *> &service_list);
  inline int send(const char *name, capnp::byte *data, size_t size) { return sockets_.at(name)->send((char *)data, size); }
  int send(const char *name, MessageBuilder &msg);
  ~PubMaster();

private:
  std::map<std::string, PubSocket *> sockets_;
};

class AlignedBuffer {
public:
  kj::ArrayPtr<const capnp::word> align(const char *data, const size_t size) {
    words_size = size / sizeof(capnp::word) + 1;
    if (aligned_buf.size() < words_size) {
      aligned_buf = kj::heapArray<capnp::word>(words_size < 512 ? 512 : words_size);
    }
    memcpy(aligned_buf.begin(), data, size);
    return aligned_buf.slice(0, words_size);
  }
  inline kj::ArrayPtr<const capnp::word> align(Message *m) {
    return align(m->getData(), m->getSize());
  }
private:
  kj::Array<capnp::word> aligned_buf;
  size_t words_size;
};

#if 0
MJ
메시징 인터페이스
이는 프로세스 간 통신(IPC)에 사용됩니다. 
PubMaster 클래스는 메시지를 발행하는 역할을 하고, 
SubMaster 클래스는 메시지를 구독하는 역할을 합니다.

주요 클래스와 그 메소드에 대한 간단한 설명은 다음과 같습니다:

Context: ZeroMQ (ZMQ)의 컨텍스트를 제공하는 클래스입니다. 
        ZMQ는 openpilot에서 IPC의 기반으로 사용되는 메시징 라이브러리입니다. 새로운 소켓을 생성하기 위한 컨텍스트가 필요합니다.

Message: 전송 또는 수신할 수 있는 단일 메시지를 나타냅니다. 
        데이터로 메시지를 초기화하고 메시지 내용에 액세스하는 메소드를 포함합니다.

SubSocket: 구독 소켓을 나타냅니다. 
        특정 토픽이나 서비스로부터 메시지를 구독하는 데 사용됩니다. 
        토픽에 연결하고, 메시지 수신 시간 제한을 설정하고, 실제 메시지를 수신하는 메소드가 포함됩니다.

PubSocket: 발행 소켓을 나타냅니다. 
        특정 토픽이나 서비스로 메시지를 보내는 데 사용됩니다. 
        토픽에 연결하고 메시지를 보내는 메소드를 포함합니다.

Poller: 하나 이상의 SubSocket을 폴링하여 수신할 준비가 된 메시지가 있는지 확인하는 데 사용됩니다.

SubMaster: SubSocket과 Poller를 사용하여 다양한 서비스에 대한 구독을 관리하는 고급 클래스입니다. 
        구독을 업데이트하고 수신된 메시지의 상태를 확인하는 메소드를 제공합니다.

MessageBuilder: 메시지를 직렬화 및 역직렬화하는 데 사용되는 Cap'n Proto 직렬화 라이브러리를 사용하여 
        메시지를 구축하기 위한 유틸리티 클래스입니다.

PubMaster: PubSocket을 사용하여 다양한 서비스로 메시지를 발행하는 고급 클래스입니다. 메시지를 보내는 메소드를 제공합니다.

AlignedBuffer: Cap'n Proto에서 예상하는 워드 경계에 데이터를 메모리에 정렬하는 데 도움을 주는 클래스입니다.

Controls 클래스 내의 self.pm은 PubMaster의 인스턴스입니다. 
        차량 제어와 관련된 다양한 메시지를 발행하는 데 사용됩니다. 
        예를 들어 'sendcan', 'controlsState', 'carState', 'carControl', 'onroadEvents', 'carParams'와 같은 정보를 포함한 메시지를 포함합니다. 
        openpilot의 아키텍처에서 이러한 메시지는 차량 제어의 원하는 상태, 차량의 현재 상태, 도로에서 발생한 이벤트 및 차량의 파라미터 등의 정보를 포함할 수 있습니다.

Controls 클래스는 self.pm을 초기화할 때 발행할 메시지의 토픽 목록을 사용하여 초기화합니다. 
        Controls 클래스가 메시지를 보내고자 할 때는 self.pm에 주제와 메시지를 보낼 메소드를 호출합니다.
#endif