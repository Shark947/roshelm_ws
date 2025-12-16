#ifndef INFO_CAST_SETTINGS_HEADER
#define INFO_CAST_SETTINGS_HEADER

class InfoCastSettings
{
public:
  InfoCastSettings();

  void setShowRealmCastSource(bool v)    {m_show_realmcast_source = v;}
  void setShowRealmCastCommunity(bool v) {m_show_realmcast_community = v;}
  void setWrapRealmCastContent(bool v)   {m_wrap_realmcast_content = v;}
  void setTruncRealmCastContent(bool v)  {m_trunc_realmcast_content = v;}
  void setRealmCastTimeFormatUTC(bool v) {m_realmcast_time_format_utc = v;}

  bool getShowRealmCastSource() const    {return(m_show_realmcast_source);}
  bool getShowRealmCastCommunity() const {return(m_show_realmcast_community);}
  bool getWrapRealmCastContent() const   {return(m_wrap_realmcast_content);}
  bool getTruncRealmCastContent() const  {return(m_trunc_realmcast_content);}
  bool getRealmCastTimeFormatUTC() const {return(m_realmcast_time_format_utc);}

private:
  bool m_show_realmcast_source;
  bool m_show_realmcast_community;
  bool m_wrap_realmcast_content;
  bool m_trunc_realmcast_content;
  bool m_realmcast_time_format_utc;
};

#endif
