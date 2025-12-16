#pragma once

#include <string>
#include <vector>

// MESSAGE TYPES
#define HELM_NOTIFY 'N'
#define HELM_REGISTER 'R'
#define HELM_UNREGISTER 'U'
#define HELM_WILDCARD_REGISTER '*'
#define HELM_WILDCARD_UNREGISTER '/'
#define HELM_NOT_SET '~'
#define HELM_COMMAND 'C'
#define HELM_ANONYMOUS 'A'
#define HELM_NULL_MSG '.'
#define HELM_DATA 'i'
#define HELM_POISON 'K'
#define HELM_WELCOME 'W'
#define HELM_SERVER_REQUEST 'Q'
#define HELM_SERVER_REQUEST_ID -2
#define HELM_TIMING 'T'
#define HELM_TERMINATE_CONNECTION '^'

// MESSAGE DATA TYPES
#define HELM_DOUBLE 'D'
#define HELM_STRING 'S'
#define HELM_BINARY_STRING 'B'

// 5 seconds time difference between client clock and DB clock will be allowed
#define HELM_SKEW_TOLERANCE 5

enum class MsgType : char
{
  Notify = HELM_NOTIFY,
  Register = HELM_REGISTER,
  Unregister = HELM_UNREGISTER,
  Command = HELM_COMMAND,
  Data = HELM_DATA,
  Unknown = HELM_NOT_SET
};

class HelmMsg
{
public:
  HelmMsg();
  virtual ~HelmMsg();

  HelmMsg(char cMsgType, const std::string &sKey, double dfVal,
          double dfTime = -1, const std::string &sSrcAux = "");
  HelmMsg(char cMsgType, const std::string &sKey, const std::string &sVal,
          double dfTime = -1, const std::string &sSrcAux = "");
  HelmMsg(char cMsgType, const std::string &sKey, unsigned int nDataSize,
          const void *Data, double dfTime = -1,
          const std::string &sSrcAux = "");

  bool operator==(const HelmMsg &M) const;

  void MarkAsBinary();

  bool IsDataType(char cDataType) const;
  bool IsDouble() const { return IsDataType(HELM_DOUBLE); }
  bool IsString() const
  {
    return IsDataType(HELM_STRING) || IsDataType(HELM_BINARY_STRING);
  }
  bool IsBinary() const { return IsDataType(HELM_BINARY_STRING); }

  bool GetBinaryData(std::vector<unsigned char> &v);
  unsigned char *GetBinaryData();
  std::vector<unsigned char> GetBinaryDataAsVector();
  unsigned int GetBinaryDataSize();

  bool IsSkewed(double dfTimeNow, double *pdfSkew = NULL);
  bool IsYoungerThan(double dfAge) const;
  bool IsType(char cType) const;
  char GetType() const;
  double GetTime() const { return m_dfTime; }
  double GetDouble() const { return m_dfVal; }
  double GetDoubleAux() const { return m_dfVal2; }
  const std::string &GetString() const { return m_sVal; }
  const std::string &GetKey() const { return m_sKey; }
  const std::string &GetName() const { return GetKey(); }
  bool IsName(const std::string &sName);
  const std::string &GetSource() const { return m_sSrc; }
  void SetSource(const std::string &sSrc) { m_sSrc = sSrc; }
  const std::string &GetSourceAux() const { return m_sSrcAux; }
  void SetSourceAux(const std::string &sSrcAux) { m_sSrcAux = sSrcAux; }
  const std::string &GetCommunity() const { return m_sOriginatingCommunity; }
  std::string GetAsString(int nFieldWidth = 12, int nNumDP = 5);
  void Trace();
  void SetDouble(double dfD) { m_dfVal = dfD; }
  void SetDoubleAux(double dfD) { m_dfVal2 = dfD; }

  char m_cMsgType;
  char m_cDataType;
  std::string m_sKey;
  int m_nID;
  double m_dfTime;
  double m_dfVal;
  double m_dfVal2;
  std::string m_sVal;
  std::string m_sSrc;
  std::string m_sSrcAux;
  std::string m_sOriginatingCommunity;

  int Serialize(unsigned char *pBuffer, int nLen, bool bToStream = true);
  bool operator<(const HelmMsg &Msg) const { return m_dfTime < Msg.m_dfTime; };
  unsigned int GetSizeInBytesWhenSerialised() const;

private:
  unsigned char *m_pSerializeBufferStart;
  unsigned char *m_pSerializeBuffer;
  int m_nSerializeBufferLen;
  int m_nLength;
  int GetLength();
  void operator<<(char &cVal);
  void operator<<(double &dfVal);
  void operator<<(std::string &sVal);
  void operator<<(int &nVal);
  void operator>>(char &cVal);
  void operator>>(double &dfVal);
  void operator>>(std::string &sVal);
  void operator>>(int &nVal);
  bool CanSerialiseN(int N);
};

using CMOOSMsg = HelmMsg;

