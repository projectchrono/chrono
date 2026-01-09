// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "SynDDSMessage.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

SynDDSMessage::SynDDSMessage()
{
    // m_rank com.eprosima.idl.parser.typecode.PrimitiveTypeCode@614ddd49
    m_rank = 0;
    // m_data com.eprosima.idl.parser.typecode.SequenceTypeCode@694e1548


}

SynDDSMessage::~SynDDSMessage()
{


}

SynDDSMessage::SynDDSMessage(
        const SynDDSMessage& x)
{
    m_rank = x.m_rank;
    m_data = x.m_data;
}

SynDDSMessage::SynDDSMessage(
        SynDDSMessage&& x)
{
    m_rank = x.m_rank;
    m_data = std::move(x.m_data);
}

SynDDSMessage& SynDDSMessage::operator =(
        const SynDDSMessage& x)
{

    m_rank = x.m_rank;
    m_data = x.m_data;

    return *this;
}

SynDDSMessage& SynDDSMessage::operator =(
        SynDDSMessage&& x)
{

    m_rank = x.m_rank;
    m_data = std::move(x.m_data);

    return *this;
}

size_t SynDDSMessage::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);




    return current_alignment - initial_alignment;
}

size_t SynDDSMessage::getCdrSerializedSize(
        const SynDDSMessage& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.data().size() > 0)
    {
        current_alignment += (data.data().size() * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }




    return current_alignment - initial_alignment;
}

void SynDDSMessage::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_rank;
    scdr << m_data;

}

void SynDDSMessage::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_rank;
    dcdr >> m_data;
}

/*!
 * @brief This function sets a value in member rank
 * @param _rank New value for member rank
 */
void SynDDSMessage::rank(
        uint32_t _rank)
{
    m_rank = _rank;
}

/*!
 * @brief This function returns the value of member rank
 * @return Value of member rank
 */
uint32_t SynDDSMessage::rank() const
{
    return m_rank;
}

/*!
 * @brief This function returns a reference to member rank
 * @return Reference to member rank
 */
uint32_t& SynDDSMessage::rank()
{
    return m_rank;
}

/*!
 * @brief This function copies the value in member data
 * @param _data New value to be copied in member data
 */
void SynDDSMessage::data(
        const std::vector<uint8_t>& _data)
{
    m_data = _data;
}

/*!
 * @brief This function moves the value in member data
 * @param _data New value to be moved in member data
 */
void SynDDSMessage::data(
        std::vector<uint8_t>&& _data)
{
    m_data = std::move(_data);
}

/*!
 * @brief This function returns a constant reference to member data
 * @return Constant reference to member data
 */
const std::vector<uint8_t>& SynDDSMessage::data() const
{
    return m_data;
}

/*!
 * @brief This function returns a reference to member data
 * @return Reference to member data
 */
std::vector<uint8_t>& SynDDSMessage::data()
{
    return m_data;
}

size_t SynDDSMessage::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;





    return current_align;
}

bool SynDDSMessage::isKeyDefined()
{
    return false;
}

void SynDDSMessage::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
      
}
