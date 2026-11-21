// One native transport node for the entire executor lifetime. Binary framing
// prevents protobuf text, escaping and large scene replies becoming shell data.
#include <gz/transport/Node.hh>
#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr std::uint32_t kMaxField = 64u * 1024u * 1024u;
bool Read32(std::uint32_t &value) {
  std::array<unsigned char, 4> data{};
  if (!std::cin.read(reinterpret_cast<char *>(data.data()), data.size()))
    return false;
  value = (std::uint32_t(data[0]) << 24) | (std::uint32_t(data[1]) << 16) |
          (std::uint32_t(data[2]) << 8) | std::uint32_t(data[3]);
  return true;
}
void Write32(std::uint32_t value) {
  const std::array<unsigned char, 4> data{{
      static_cast<unsigned char>(value >> 24),
      static_cast<unsigned char>(value >> 16),
      static_cast<unsigned char>(value >> 8),
      static_cast<unsigned char>(value)}};
  std::cout.write(reinterpret_cast<const char *>(data.data()), data.size());
}
std::string ReadField(std::uint32_t length) {
  if (length > kMaxField) throw std::runtime_error("request field exceeds 64 MiB");
  std::string result(length, '\0');
  if (!std::cin.read(result.data(), length))
    throw std::runtime_error("incomplete request frame");
  return result;
}
void Reply(std::uint32_t status, const std::string &payload) {
  if (payload.size() > kMaxField)
    throw std::runtime_error("reply exceeds 64 MiB");
  Write32(status);
  Write32(static_cast<std::uint32_t>(payload.size()));
  std::cout.write(payload.data(), payload.size());
  std::cout.flush();
}
}  // namespace

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  gz::transport::Node node;
  while (true) {
    std::uint32_t timeout = 0;
    if (!Read32(timeout)) return std::cin.eof() ? 0 : 2;
    try {
      std::array<std::uint32_t, 4> lengths{};
      for (auto &length : lengths)
        if (!Read32(length)) throw std::runtime_error("incomplete frame header");
      const auto service = ReadField(lengths[0]);
      const auto requestType = ReadField(lengths[1]);
      const auto responseType = ReadField(lengths[2]);
      const auto request = ReadField(lengths[3]);
      if (timeout == 0) throw std::runtime_error("request timeout must be positive");
      std::string response;
      bool result = false;
      const bool executed = node.RequestRaw(service, request, requestType,
                                            responseType, timeout, response, result);
      if (!executed) Reply(1, "Gazebo service reply timed out");
      else if (!result) Reply(2, "Gazebo service explicitly rejected request");
      else Reply(0, response);
    } catch (const std::exception &error) {
      Reply(3, error.what());
      return 2;  // A malformed frame cannot be safely resynchronized.
    }
  }
}
