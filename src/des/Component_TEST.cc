/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * - Neither the name of prim nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "des/Component.h"

#include "des/Simulator.h"
#include "gtest/gtest.h"

namespace {
class TestComponent : public des::Component {
 public:
  TestComponent(const std::string& _name, const Component* _parent)
      : des::Component(_name, _parent) {}
  TestComponent(des::Simulator* _simulator, const std::string& _name)
      : des::Component(_simulator, _name) {}
  ~TestComponent() {}
};
}  // namespace

TEST(Component, construct) {
  des::Simulator sim;

  TestComponent tc1(&sim, "top");
  ASSERT_EQ(tc1.basename(), "top");
  ASSERT_EQ(tc1.fullname(), "top");
  ASSERT_EQ(tc1.parent(), nullptr);
  ASSERT_EQ(tc1.simulator, &sim);

  TestComponent tc2("left", &tc1);
  ASSERT_EQ(tc2.basename(), "left");
  ASSERT_EQ(tc2.fullname(), "top.left");
  ASSERT_EQ(tc2.parent(), &tc1);
  ASSERT_EQ(tc2.simulator, &sim);

  TestComponent tc3("right", &tc1);
  ASSERT_EQ(tc3.basename(), "right");
  ASSERT_EQ(tc3.fullname(), "top.right");
  ASSERT_EQ(tc3.parent(), &tc1);
  ASSERT_EQ(tc3.simulator, &sim);
}
