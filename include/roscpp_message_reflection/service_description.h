#ifndef ROSCPP_MESSAGE_REFLECTION_SERVICE_DESCRIPTION_H
#define ROSCPP_MESSAGE_REFLECTION_SERVICE_DESCRIPTION_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <roscpp_message_reflection/message_description.h>

namespace roscpp_message_reflection {

struct ServiceDescription {
  typedef boost::shared_ptr<ServiceDescription> Ptr;

  ServiceDescription(const std::string& name, const std::string& md5sum,
		     MessageDescription::Ptr request, MessageDescription::Ptr response)
    : name(name), md5sum(md5sum), request(request), response(response) {}

  const std::string name;
  const std::string md5sum;
  const MessageDescription::Ptr request;
  const MessageDescription::Ptr response;
};

}

#endif
