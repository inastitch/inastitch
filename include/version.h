// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace inastitch
{
  struct version
  {
    static const std::string GIT_COMMIT_TAG;
    static const std::string GIT_COMMIT_DATE;
  };
}
