/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>

#include <llvm/raw_ostream.h>

#include "cscore.h"

int main() {
  llvm::outs() << "hostname: " << cs::GetHostname() << '\n';
  llvm::outs() << "IPv4 network addresses:\n";
  for (const auto& addr : cs::GetNetworkInterfaces())
    llvm::outs() << "  " << addr << '\n';
  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 160, 120, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);

  std::getchar();
}
